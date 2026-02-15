#!/bin/bash

echo "╔════════════════════════════════════════════════════════════╗"
echo "║        Earth Rover - Service Status Check                 ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Django VCS
echo "🌐 Django VCS (port 8000):"
if netstat -tuln 2>/dev/null | grep -q ':8000 ' || ss -tuln 2>/dev/null | grep -q ':8000 '; then
    echo "   ✓ RUNNING"
    PID=$(pgrep -f "manage.py runserver" | head -1)
    [[ -n "$PID" ]] && echo "   PID: $PID"
else
    echo "   ✗ NOT RUNNING"
fi
echo ""

# Web Video Server
echo "📹 Web Video Server (port 8080):"
if netstat -tuln 2>/dev/null | grep -q ':8080 ' || ss -tuln 2>/dev/null | grep -q ':8080 '; then
    echo "   ✓ RUNNING"
    PID=$(pgrep -f "web_video_server" | head -1)
    [[ -n "$PID" ]] && echo "   PID: $PID"
else
    echo "   ✗ NOT RUNNING"
fi
echo ""

# ROSBridge
echo "🔌 ROSBridge WebSocket (port 9090):"
if netstat -tuln 2>/dev/null | grep -q ':9090 ' || ss -tuln 2>/dev/null | grep -q ':9090 '; then
    echo "   ✓ RUNNING"
    PID=$(pgrep -f "rosbridge" | head -1)
    [[ -n "$PID" ]] && echo "   PID: $PID"
else
    echo "   ✗ NOT RUNNING"
fi
echo ""

# ROS Nodes
echo "🤖 ROS2 Nodes:"
if command -v ros2 &> /dev/null; then
    NODE_COUNT=$(ros2 node list 2>/dev/null | wc -l)
    if [ "$NODE_COUNT" -gt 0 ]; then
        echo "   ✓ $NODE_COUNT nodes active"
        echo "   Nodes: $(ros2 node list 2>/dev/null | tr '\n' ', ' | sed 's/,$//')"
    else
        echo "   ⚠ No nodes detected"
    fi
else
    echo "   ✗ ROS2 not found"
fi
echo ""

# Logs
echo "📝 Recent Log Activity:"
LOG_DIR="/home/jdas/earth-rover/scripts/logs"
if [ -d "$LOG_DIR" ]; then
    for log in django_vcs rosbridge web_video_server; do
        if [ -f "$LOG_DIR/${log}.log" ]; then
            SIZE=$(du -h "$LOG_DIR/${log}.log" | cut -f1)
            MODIFIED=$(stat -c %y "$LOG_DIR/${log}.log" 2>/dev/null | cut -d. -f1)
            echo "   ${log}.log: $SIZE (modified: $MODIFIED)"
        fi
    done
else
    echo "   ⚠ Log directory not found"
fi
echo ""

# Network Connections
echo "🌍 Active Connections to VCS:"
CONN_COUNT=$(netstat -an 2>/dev/null | grep ':8000.*ESTABLISHED' | wc -l)
echo "   $CONN_COUNT active connection(s)"
if [ "$CONN_COUNT" -gt 0 ]; then
    netstat -an 2>/dev/null | grep ':8000.*ESTABLISHED' | awk '{print "   - " $5}' | sed 's/:/ (port /'  | sed 's/$/)/'
fi

echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║  Access VCS: http://localhost:8000                         ║"
echo "║  Access VCS: http://192.168.1.7:8000                       ║"
echo "╚════════════════════════════════════════════════════════════╝"
