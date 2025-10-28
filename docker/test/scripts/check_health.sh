#!/bin/bash
# Check health of all services in test environment

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

echo -e "${BLUE}🏥 Service Health Check${NC}"
echo "======================"
echo ""

# Get all services from docker-compose
SERVICES=$(docker compose -f docker-compose-x86-test.yaml config --services)

TOTAL=0
RUNNING=0
EXITED=0
NOT_STARTED=0

for service in $SERVICES; do
    TOTAL=$((TOTAL + 1))
    
    # Get container name and status
    container_info=$(docker compose -f docker-compose-x86-test.yaml ps "$service" 2>/dev/null | tail -n +2)
    
    if [ -z "$container_info" ]; then
        # Not started
        NOT_STARTED=$((NOT_STARTED + 1))
        printf "%-30s ${YELLOW}⚠ Not Started${NC}\n" "$service"
    elif echo "$container_info" | grep -q "Up"; then
        # Running
        RUNNING=$((RUNNING + 1))
        
        # Check health status if available
        health=$(echo "$container_info" | grep -o "healthy\|unhealthy\|starting" || echo "")
        if [ -n "$health" ]; then
            case "$health" in
                "healthy")
                    printf "%-30s ${GREEN}✓ Running (healthy)${NC}\n" "$service"
                    ;;
                "unhealthy")
                    printf "%-30s ${RED}✗ Running (unhealthy)${NC}\n" "$service"
                    ;;
                "starting")
                    printf "%-30s ${YELLOW}⟳ Running (starting)${NC}\n" "$service"
                    ;;
            esac
        else
            printf "%-30s ${GREEN}✓ Running${NC}\n" "$service"
        fi
    else
        # Exited
        EXITED=$((EXITED + 1))
        exit_code=$(echo "$container_info" | grep -o "Exited ([0-9]*)" | grep -o "[0-9]*")
        if [ -z "$exit_code" ]; then
            printf "%-30s ${RED}✗ Exited${NC}\n" "$service"
        else
            printf "%-30s ${RED}✗ Exited ($exit_code)${NC}\n" "$service"
        fi
    fi
done

echo ""
echo "Summary:"
echo "  Total services:    $TOTAL"
echo "  Running:          ${GREEN}$RUNNING${NC}"
echo "  Exited:           ${RED}$EXITED${NC}"
echo "  Not started:      ${YELLOW}$NOT_STARTED${NC}"

echo ""
echo "Checking Zenoh router connectivity..."
if curl -f -s http://localhost:8000/@/local/router &>/dev/null; then
    echo -e "  ${GREEN}✓ Zenoh REST API responding${NC}"
    
    # Get router stats
    stats=$(curl -s http://localhost:8000/@/router/local)
    if [ -n "$stats" ]; then
        echo ""
        echo "Zenoh Router Stats:"
        echo "$stats" | jq -r '
            "  Zenoh ID: " + (.zid // "unknown"),
            "  Version:  " + (.version // "unknown"),
            "  Locators: " + ((.locators // []) | join(", "))
        ' 2>/dev/null || echo "  (Could not parse stats)"
    fi
else
    echo -e "  ${RED}✗ Zenoh REST API not responding${NC}"
fi

echo ""

# Show recent errors from logs
echo "Recent errors from logs (last 10 lines per service):"
echo "=================================================="
for service in $SERVICES; do
    if docker compose -f docker-compose-x86-test.yaml ps "$service" 2>/dev/null | tail -n +2 | grep -q .; then
        errors=$(docker compose -f docker-compose-x86-test.yaml logs --tail=10 "$service" 2>&1 | grep -i "error\|fatal\|exception" || true)
        if [ -n "$errors" ]; then
            echo ""
            echo -e "${RED}$service:${NC}"
            echo "$errors" | head -5 | sed 's/^/  /'
        fi
    fi
done

echo ""
echo "For detailed logs: docker compose -f docker-compose-x86-test.yaml logs -f [service]"
echo ""
