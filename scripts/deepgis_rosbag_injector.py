#!/usr/bin/env python3
"""
DeepGIS Rosbag GPS Injector

Reads GPS data from a ROS2 rosbag file and injects it into the DeepGIS database
via the DeepGIS Telemetry API. Follows the same API compliance as deepgis_gps_publisher.py.

Usage:
    python3 deepgis_rosbag_injector.py --bag-path /path/to/rosbag
    python3 deepgis_rosbag_injector.py --bag-path /path/to/rosbag --rate 10.0 --asset-name "MyRover"
"""

import argparse
import math
import re
import requests
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional, Tuple, Dict, Any

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import UInt32
except ImportError as e:
    print(f"Error importing ROS2 packages: {e}")
    print("Make sure you have sourced your ROS2 workspace:")
    print("  source /opt/ros/humble/setup.bash")
    print("  source ~/ros2_ws/install/setup.bash")
    sys.exit(1)


class DeepGISRosbagInjector:
    """
    Reads GPS data from a ROS2 rosbag and injects it into DeepGIS API.
    """

    def __init__(
        self,
        bag_path: str,
        api_url: str = 'https://deepgis.org',
        api_key: str = 'test-missions',
        asset_name: str = 'R/V Karin Valentine (robotic boat)',
        session_id: Optional[str] = None,
        project_title: str = 'Collaborative Robotic Aquatic Laboratory (CoRAL)',
        flight_mode: str = 'AUTO',
        mission_type: str = 'weekly-test-mission',
        notes: str = 'Automated GPS telemetry import from rosbag',
        rate_limit: float = 0.0,  # Messages per second (0 = no limit)
        dry_run: bool = False,
        verbose: bool = False,
    ):
        self.bag_path = Path(bag_path)
        self.api_url = api_url
        self.api_key = api_key
        self.asset_name = asset_name
        
        # Extract date from rosbag folder name
        self.rosbag_datetime = self._extract_rosbag_datetime()
        
        # Use rosbag date for session_id if not provided
        if session_id:
            self.session_id = session_id
        elif self.rosbag_datetime:
            self.session_id = f"rosbag_{self.rosbag_datetime.strftime('%Y%m%d_%H%M%S')}"
        else:
            self.session_id = f"rosbag_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        
        # Generate human-readable session name from rosbag date
        if self.rosbag_datetime:
            self.session_name = f"Mission {self.rosbag_datetime.strftime('%Y-%m-%d %H:%M:%S')}"
        else:
            self.session_name = f"Mission {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}"
        
        self.project_title = project_title
        self.flight_mode = flight_mode
        self.mission_type = mission_type
        self.notes = notes
        self.rate_limit = rate_limit
        self.dry_run = dry_run
        self.verbose = verbose

        # API endpoints
        self.api_endpoints = {
            'create_session': f'{self.api_url}/api/telemetry/session/create/',
            'gps_raw': f'{self.api_url}/api/telemetry/gps-fix-raw/',
            'gps_estimated': f'{self.api_url}/api/telemetry/gps-fix-estimated/',
        }

        # Session state
        self.session_active = False

        # Satellite count (shared between messages)
        self.satellites_visible = 0

        # Statistics
        self.stats = {
            'gps_raw_sent': 0,
            'gps_raw_failed': 0,
            'gps_estimated_sent': 0,
            'gps_estimated_failed': 0,
            'satellites_processed': 0,
        }

        # HTTP session with connection pooling
        self.http_session = requests.Session()
        if self.api_key:
            self.http_session.headers.update({'Authorization': f'Bearer {self.api_key}'})
        self.http_session.headers.update({'Content-Type': 'application/json'})

        # Topic mappings
        self.gps_raw_topic = '/mavros/global_position/raw/fix'
        self.gps_estimated_topic = '/mavros/global_position/global'
        self.satellites_topic = '/mavros/global_position/raw/satellites'

        # Validate bag path
        if not self.bag_path.exists():
            raise FileNotFoundError(f"Rosbag path does not exist: {self.bag_path}")

    def _extract_rosbag_datetime(self) -> Optional[datetime]:
        """
        Extract datetime from rosbag folder name.
        
        Supports formats like:
        - rosbag2_2025_10_25_20_47_07 -> 2025-10-25 20:47:07
        - rosbag2_2025_10_25-20_47_07
        - 2025_10_25_20_47_07
        
        Returns:
            datetime object if successfully parsed, None otherwise
        """
        folder_name = self.bag_path.name
        
        # Pattern: rosbag2_YYYY_MM_DD_HH_MM_SS or similar
        patterns = [
            r'(\d{4})_(\d{2})_(\d{2})_(\d{2})_(\d{2})_(\d{2})',  # YYYY_MM_DD_HH_MM_SS
            r'(\d{4})-(\d{2})-(\d{2})_(\d{2})_(\d{2})_(\d{2})',  # YYYY-MM-DD_HH_MM_SS
            r'(\d{4})_(\d{2})_(\d{2})-(\d{2})_(\d{2})_(\d{2})',  # YYYY_MM_DD-HH_MM_SS
        ]
        
        for pattern in patterns:
            match = re.search(pattern, folder_name)
            if match:
                try:
                    year, month, day, hour, minute, second = map(int, match.groups())
                    return datetime(year, month, day, hour, minute, second)
                except ValueError:
                    continue
        
        return None

    def _ros_timestamp_to_datetime(self, stamp) -> datetime:
        """
        Convert ROS timestamp (header.stamp) to datetime object.
        
        Args:
            stamp: ROS timestamp with sec and nanosec fields
            
        Returns:
            datetime object
        """
        # Convert ROS timestamp to seconds since epoch
        timestamp_sec = stamp.sec + stamp.nanosec / 1e9
        return datetime.fromtimestamp(timestamp_sec, tz=timezone.utc)

    def log(self, msg: str, level: str = 'INFO'):
        """Log a message with timestamp."""
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        print(f"[{timestamp}] [{level}] {msg}")

    def log_verbose(self, msg: str):
        """Log a message only in verbose mode."""
        if self.verbose:
            self.log(msg, 'DEBUG')

    def log_response(self, response: requests.Response, endpoint_name: str, request_url: Optional[str] = None):
        """Log server response (success or error) from DeepGIS server."""
        url = request_url or (response.url if hasattr(response, 'url') else 'unknown')
        is_success = response.status_code in [200, 201]
        
        if is_success:
            msg = f'[{endpoint_name}] Request successful'
        else:
            msg = f'[{endpoint_name}] Request failed'
        
        msg += f'\n  URL: {url}'
        msg += f'\n  Status Code: {response.status_code}'
        msg += f'\n  Status Text: {response.reason if hasattr(response, "reason") else "N/A"}'
        
        # Try to parse JSON response
        try:
            response_json = response.json()
            msg += f'\n  Response JSON: {json.dumps(response_json, indent=2)}'
        except (ValueError, json.JSONDecodeError):
            response_text = response.text[:500]
            msg += f'\n  Response Text: {response_text}'
            if len(response.text) > 500:
                msg += '... (truncated)'
        
        if is_success:
            self.log(msg)
        else:
            self.log(msg, 'ERROR')

    def create_telemetry_session(self) -> bool:
        """Create a new telemetry session with DeepGIS API."""
        if self.dry_run:
            self.log(f'[DRY RUN] Would create telemetry session: {self.session_id} ({self.session_name})')
            self.session_active = True
            return True

        try:
            payload = {
                'session_id': self.session_id,
                'session_name': self.session_name,
                'asset_name': self.asset_name,
                'project_title': self.project_title,
                'flight_mode': self.flight_mode,
                'mission_type': self.mission_type,
                'notes': self.notes
            }

            response = self.http_session.post(
                self.api_endpoints['create_session'],
                json=payload,
                timeout=10.0
            )

            if response.status_code in [200, 201]:
                self.session_active = True
                self.log(f'Created telemetry session: {self.session_id}')
                self.log_response(response, 'create_session', self.api_endpoints['create_session'])
                return True
            else:
                self.log_response(response, 'create_session', self.api_endpoints['create_session'])
                return False

        except requests.exceptions.RequestException as e:
            self.log(f'Error creating telemetry session: {str(e)}', 'ERROR')
            return False

    def extract_gps_accuracy(self, msg: NavSatFix) -> Tuple[float, float]:
        """Extract horizontal and vertical accuracy from GPS covariance."""
        if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            return 99.99, 99.99
        
        # Extract variances (diagonal elements)
        var_lat = msg.position_covariance[0]
        var_lon = msg.position_covariance[4]
        var_alt = msg.position_covariance[8]
        
        # Convert to standard deviations
        eph = math.sqrt((var_lat + var_lon) / 2.0)
        epv = math.sqrt(var_alt)
        
        return float(eph), float(epv)

    def map_gps_fix_type(self, status: int) -> int:
        """Map NavSatStatus to GPS fix type."""
        if status < 0:
            return 0  # No fix
        elif status == 0:
            return 3  # 3D fix
        elif status == 1:
            return 4  # SBAS (augmented)
        elif status == 2:
            return 4  # GBAS
        else:
            return 3  # Default to 3D

    def format_gps_data(self, msg: NavSatFix, is_raw: bool = True) -> Dict[str, Any]:
        """Format GPS fix data for DeepGIS API using actual GPS timestamp."""
        eph, epv = self.extract_gps_accuracy(msg)
        fix_type = self.map_gps_fix_type(msg.status.status)
        
        # Convert ROS timestamp to microseconds
        timestamp_usec = msg.header.stamp.sec * 1000000 + msg.header.stamp.nanosec // 1000
        
        # Use actual GPS timestamp from the rosbag data
        gps_datetime = self._ros_timestamp_to_datetime(msg.header.stamp)
        
        return {
            'session_id': self.session_id,
            'timestamp': gps_datetime.isoformat(),
            'timestamp_usec': timestamp_usec,
            'latitude': float(msg.latitude),
            'longitude': float(msg.longitude),
            'altitude': float(msg.altitude),
            'fix_type': fix_type,
            'eph': float(eph),
            'epv': float(epv),
            'satellites_visible': int(self.satellites_visible)
        }

    def send_gps_raw(self, data: Dict[str, Any]) -> bool:
        """Send raw GPS data to API."""
        if self.dry_run:
            self.log_verbose(f'[DRY RUN] Would send GPS raw: lat={data["latitude"]:.6f}, lon={data["longitude"]:.6f}, time={data["timestamp"]}')
            self.stats['gps_raw_sent'] += 1
            return True

        try:
            response = self.http_session.post(
                self.api_endpoints['gps_raw'],
                json=data,
                timeout=5.0
            )
            if response.status_code in [200, 201]:
                self.stats['gps_raw_sent'] += 1
                self.log_verbose(f'Sent GPS raw: lat={data["latitude"]:.6f}, lon={data["longitude"]:.6f}, time={data["timestamp"]}')
                return True
            else:
                self.stats['gps_raw_failed'] += 1
                if self.verbose:
                    self.log_response(response, 'gps_raw', self.api_endpoints['gps_raw'])
                return False
        except requests.exceptions.RequestException as e:
            self.stats['gps_raw_failed'] += 1
            self.log(f'Error sending GPS raw: {str(e)}', 'WARN')
            return False

    def send_gps_estimated(self, data: Dict[str, Any]) -> bool:
        """Send estimated GPS data to API."""
        if self.dry_run:
            self.log_verbose(f'[DRY RUN] Would send GPS estimated: lat={data["latitude"]:.6f}, lon={data["longitude"]:.6f}, time={data["timestamp"]}')
            self.stats['gps_estimated_sent'] += 1
            return True

        try:
            response = self.http_session.post(
                self.api_endpoints['gps_estimated'],
                json=data,
                timeout=5.0
            )
            if response.status_code in [200, 201]:
                self.stats['gps_estimated_sent'] += 1
                self.log_verbose(f'Sent GPS estimated: lat={data["latitude"]:.6f}, lon={data["longitude"]:.6f}, time={data["timestamp"]}')
                return True
            else:
                self.stats['gps_estimated_failed'] += 1
                if self.verbose:
                    self.log_response(response, 'gps_estimated', self.api_endpoints['gps_estimated'])
                return False
        except requests.exceptions.RequestException as e:
            self.stats['gps_estimated_failed'] += 1
            self.log(f'Error sending GPS estimated: {str(e)}', 'WARN')
            return False

    def process_rosbag(self) -> bool:
        """Process the rosbag and send GPS data to DeepGIS API."""
        self.log(f'Opening rosbag: {self.bag_path}')
        
        # Create reader
        reader = SequentialReader()
        
        # Determine storage format
        bag_dir = self.bag_path
        mcap_files = list(bag_dir.glob('*.mcap')) if bag_dir.is_dir() else []
        db3_files = list(bag_dir.glob('*.db3')) if bag_dir.is_dir() else []
        
        if bag_dir.is_file():
            # Single file provided
            if str(bag_dir).endswith('.mcap'):
                storage_id = 'mcap'
            else:
                storage_id = 'sqlite3'
            storage_uri = str(bag_dir.parent)
        elif mcap_files:
            storage_id = 'mcap'
            storage_uri = str(bag_dir)
        elif db3_files:
            storage_id = 'sqlite3'
            storage_uri = str(bag_dir)
        else:
            # Try mcap first (more common in newer ROS2)
            storage_id = 'mcap'
            storage_uri = str(bag_dir)
        
        self.log(f'Using storage: {storage_id}, URI: {storage_uri}')
        
        storage_options = StorageOptions(uri=storage_uri, storage_id=storage_id)
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        
        try:
            reader.open(storage_options, converter_options)
        except Exception as e:
            self.log(f'Error opening rosbag: {e}', 'ERROR')
            return False
        
        # Get topic info
        topic_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topic_types}
        
        self.log(f'Found {len(topic_types)} topics in rosbag')
        
        # Check for GPS topics
        gps_topics_found = []
        if self.gps_raw_topic in type_map:
            gps_topics_found.append(self.gps_raw_topic)
        if self.gps_estimated_topic in type_map:
            gps_topics_found.append(self.gps_estimated_topic)
        if self.satellites_topic in type_map:
            gps_topics_found.append(self.satellites_topic)
        
        if not gps_topics_found:
            self.log('No GPS topics found in rosbag!', 'ERROR')
            return False
        
        self.log(f'Found GPS topics: {gps_topics_found}')
        
        # Create session
        if not self.create_telemetry_session():
            self.log('Failed to create telemetry session', 'ERROR')
            return False
        
        # Process messages
        message_count = 0
        last_send_time = 0
        min_interval = 1.0 / self.rate_limit if self.rate_limit > 0 else 0
        
        self.log('Processing rosbag messages...')
        start_time = time.time()
        
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            
            # Rate limiting
            if self.rate_limit > 0:
                current_time = time.time()
                elapsed = current_time - last_send_time
                if elapsed < min_interval:
                    time.sleep(min_interval - elapsed)
                last_send_time = time.time()
            
            try:
                if topic == self.satellites_topic:
                    msg = deserialize_message(data, UInt32)
                    self.satellites_visible = msg.data
                    self.stats['satellites_processed'] += 1
                    
                elif topic == self.gps_raw_topic:
                    msg = deserialize_message(data, NavSatFix)
                    formatted_data = self.format_gps_data(msg, is_raw=True)
                    self.send_gps_raw(formatted_data)
                    message_count += 1
                    
                elif topic == self.gps_estimated_topic:
                    msg = deserialize_message(data, NavSatFix)
                    formatted_data = self.format_gps_data(msg, is_raw=False)
                    self.send_gps_estimated(formatted_data)
                    message_count += 1
                    
            except Exception as e:
                self.log(f'Error processing message on {topic}: {e}', 'WARN')
            
            # Progress update every 1000 messages
            if message_count > 0 and message_count % 1000 == 0:
                elapsed = time.time() - start_time
                rate = message_count / elapsed if elapsed > 0 else 0
                self.log(f'Processed {message_count} GPS messages ({rate:.1f} msg/s)')
        
        elapsed = time.time() - start_time
        self.log(f'Finished processing rosbag in {elapsed:.1f} seconds')
        
        return True

    def print_summary(self):
        """Print a summary of the injection process."""
        self.log('=' * 60)
        self.log('INJECTION SUMMARY')
        self.log('=' * 60)
        self.log(f'Session ID: {self.session_id}')
        self.log(f'Session Name: {self.session_name}')
        self.log(f'Asset Name: {self.asset_name}')
        self.log(f'Project Title: {self.project_title}')
        self.log(f'Mission Type: {self.mission_type}')
        self.log(f'Rosbag: {self.bag_path}')
        if self.rosbag_datetime:
            self.log(f'Rosbag Date: {self.rosbag_datetime.strftime("%Y-%m-%d %H:%M:%S")}')
        self.log('-' * 60)
        self.log(f'GPS Raw Sent: {self.stats["gps_raw_sent"]}')
        self.log(f'GPS Raw Failed: {self.stats["gps_raw_failed"]}')
        self.log(f'GPS Estimated Sent: {self.stats["gps_estimated_sent"]}')
        self.log(f'GPS Estimated Failed: {self.stats["gps_estimated_failed"]}')
        self.log(f'Satellites Updates: {self.stats["satellites_processed"]}')
        self.log('-' * 60)
        total_sent = self.stats['gps_raw_sent'] + self.stats['gps_estimated_sent']
        total_failed = self.stats['gps_raw_failed'] + self.stats['gps_estimated_failed']
        self.log(f'Total Messages Sent: {total_sent}')
        self.log(f'Total Messages Failed: {total_failed}')
        if total_sent + total_failed > 0:
            success_rate = total_sent / (total_sent + total_failed) * 100
            self.log(f'Success Rate: {success_rate:.1f}%')
        self.log('=' * 60)

    def run(self) -> bool:
        """Run the rosbag injection process."""
        self.log('=' * 60)
        self.log('DeepGIS Rosbag GPS Injector')
        self.log('=' * 60)
        self.log(f'API URL: {self.api_url}')
        self.log(f'Asset Name: {self.asset_name}')
        self.log(f'Project Title: {self.project_title}')
        self.log(f'Mission Type: {self.mission_type}')
        self.log(f'Flight Mode: {self.flight_mode}')
        self.log(f'Session ID: {self.session_id}')
        self.log(f'Session Name: {self.session_name}')
        self.log(f'Notes: {self.notes}')
        self.log(f'Rosbag Path: {self.bag_path}')
        if self.rosbag_datetime:
            self.log(f'Rosbag Date: {self.rosbag_datetime.strftime("%Y-%m-%d %H:%M:%S")}')
        else:
            self.log('Rosbag Date: Could not extract from folder name')
        if self.rate_limit > 0:
            self.log(f'Rate Limit: {self.rate_limit} msg/s')
        else:
            self.log('Rate Limit: None (full speed)')
        self.log('=' * 60)
        if self.dry_run:
            self.log('*** DRY RUN MODE - No data will be sent ***')
        
        try:
            success = self.process_rosbag()
            self.print_summary()
            return success
        except KeyboardInterrupt:
            self.log('Interrupted by user', 'WARN')
            self.print_summary()
            return False
        except Exception as e:
            self.log(f'Error: {e}', 'ERROR')
            import traceback
            traceback.print_exc()
            return False
        finally:
            self.http_session.close()


def main():
    parser = argparse.ArgumentParser(
        description='Inject GPS data from a ROS2 rosbag into DeepGIS database',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Basic usage (uses CoRAL project defaults)
    python3 deepgis_rosbag_injector.py --bag-path ~/Downloads/rosbag2_2025_10_25_20_47_07

    # With custom settings
    python3 deepgis_rosbag_injector.py \\
        --bag-path ~/Downloads/rosbag2_2025_10_25_20_47_07 \\
        --asset-name "R/V Karin Valentine (robotic boat)" \\
        --project-title "Collaborative Robotic Aquatic Laboratory (CoRAL)" \\
        --mission-type "weekly-test-mission" \\
        --rate 10.0

    # Dry run to test without sending data
    python3 deepgis_rosbag_injector.py \\
        --bag-path ~/Downloads/rosbag2_2025_10_25_20_47_07 \\
        --dry-run --verbose

    # With custom session ID and notes
    python3 deepgis_rosbag_injector.py \\
        --bag-path ~/Downloads/rosbag2_2025_10_25_20_47_07 \\
        --session-id "tempe_town_lake_20251025" \\
        --notes "Water quality survey mission at Tempe Town Lake"
        """
    )
    
    parser.add_argument(
        '--bag-path', '-b',
        type=str,
        required=True,
        help='Path to the ROS2 rosbag directory or file'
    )
    parser.add_argument(
        '--api-url',
        type=str,
        default='https://deepgis.org',
        help='DeepGIS API URL (default: https://deepgis.org)'
    )
    parser.add_argument(
        '--api-key',
        type=str,
        default='test-missions',
        help='DeepGIS API key (default: test-missions)'
    )
    parser.add_argument(
        '--asset-name', '-a',
        type=str,
        default='R/V Karin Valentine (robotic boat)',
        help='Asset name (default: R/V Karin Valentine (robotic boat))'
    )
    parser.add_argument(
        '--session-id', '-s',
        type=str,
        default=None,
        help='Session ID (auto-generated if not provided)'
    )
    parser.add_argument(
        '--project-title', '-p',
        type=str,
        default='Collaborative Robotic Aquatic Laboratory (CoRAL)',
        help='Project title (default: Collaborative Robotic Aquatic Laboratory (CoRAL))'
    )
    parser.add_argument(
        '--flight-mode',
        type=str,
        default='AUTO',
        help='Flight mode (default: AUTO)'
    )
    parser.add_argument(
        '--mission-type',
        type=str,
        default='weekly-test-mission',
        help='Mission type (default: weekly-test-mission)'
    )
    parser.add_argument(
        '--notes',
        type=str,
        default='Automated GPS telemetry import from rosbag',
        help='Notes for the session'
    )
    parser.add_argument(
        '--rate', '-r',
        type=float,
        default=0.0,
        help='Rate limit in messages per second (0 = no limit, default: 0)'
    )
    parser.add_argument(
        '--dry-run', '-d',
        action='store_true',
        help='Dry run mode - parse rosbag but do not send to API'
    )
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Verbose output'
    )
    
    args = parser.parse_args()
    
    injector = DeepGISRosbagInjector(
        bag_path=args.bag_path,
        api_url=args.api_url,
        api_key=args.api_key,
        asset_name=args.asset_name,
        session_id=args.session_id,
        project_title=args.project_title,
        flight_mode=args.flight_mode,
        mission_type=args.mission_type,
        notes=args.notes,
        rate_limit=args.rate,
        dry_run=args.dry_run,
        verbose=args.verbose,
    )
    
    success = injector.run()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()

