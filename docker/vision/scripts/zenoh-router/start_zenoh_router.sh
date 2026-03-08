#!/bin/bash
set -e

echo "=========================================="
echo "  Vision Zenoh Router"
echo "=========================================="
echo "Config: /config/zenoh_router_config.json5"

exec /zenohd -c /config/zenoh_router_config.json5
