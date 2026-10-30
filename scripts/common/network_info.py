#!/usr/bin/env python3
"""
F1TENTH Network Endpoint & URL Resolution Utility
Handles URL resolution for local host, mDNS discovery, and pitlane LAN IP.
Works seamlessly across macOS, Linux, Jetson Orin, and Docker containers.
"""

import os
import re
import socket
from typing import Dict, List, Optional


def find_repo_root() -> str:
    curr = os.path.dirname(os.path.abspath(__file__))
    while curr != os.path.dirname(curr):
        if os.path.isfile(os.path.join(curr, ".env")) or os.path.isdir(os.path.join(curr, "src")):
            return curr
        curr = os.path.dirname(curr)
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def get_network_endpoints(port: int, query: str = "") -> Dict[str, str]:
    """
    Resolves access URLs for local (same machine), mDNS (from Pit Mac/external),
    and pitlane LAN IP.
    """
    clean_query = f"?{query.lstrip('?')}" if query else ""

    # 1. Local URL (same machine or SSH port forwarding)
    local_url = f"http://localhost:{port}/{clean_query}"

    # 2. Hostname resolution
    raw_hostname = socket.gethostname()
    if raw_hostname.endswith(".local"):
        raw_hostname = raw_hostname[:-6]

    is_docker = os.path.exists("/.dockerenv")
    # Check if hostname looks like a 12-char hex Docker container ID (e.g. 0c74bdf651fb)
    is_docker_hex = bool(re.match(r"^[0-9a-f]{12}$", raw_hostname.lower()))

    # Read CAR_HOSTNAME from .env or environment
    car_hostname = os.environ.get("CAR_HOSTNAME")
    if not car_hostname:
        try:
            repo_root = find_repo_root()
            env_file = os.path.join(repo_root, ".env")
            if os.path.exists(env_file):
                with open(env_file, "r") as f:
                    for line in f:
                        line = line.strip()
                        if line.startswith("CAR_HOSTNAME="):
                            car_hostname = line.split("=", 1)[1].split("#")[0].strip()
                            break
        except Exception:
            pass

    if (is_docker_hex or is_docker) and car_hostname:
        mdns_host = car_hostname[:-6] if car_hostname.endswith(".local") else car_hostname
    else:
        mdns_host = raw_hostname

    mdns_url = f"http://{mdns_host}.local:{port}/{clean_query}"

    # 3. Resolve actual LAN IP
    host_ip = None
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        host_ip = s.getsockname()[0]
        s.close()
    except Exception:
        try:
            host_ip = socket.gethostbyname(raw_hostname)
        except Exception:
            host_ip = "127.0.0.1"

    pitlane_ip = host_ip or "<jetson_or_host_ip>"
    lan_url = f"http://{pitlane_ip}:{port}/{clean_query}"

    return {
        "local": local_url,
        "mdns": mdns_url,
        "lan": lan_url,
        "is_docker": is_docker,
        "hostname": mdns_host,
        "ip": pitlane_ip
    }


def print_service_banner(
    title: str,
    port: int,
    query: str = "",
    subtitle: Optional[str] = None,
    extra_lines: Optional[List[str]] = None,
    width: int = 75
):
    endpoints = get_network_endpoints(port, query)

    print("\n" + "=" * width)
    print(f"  {title}")
    if subtitle:
        print(f"  {subtitle}")
    print("=" * width)
    print(f"  → Same Machine:    {endpoints['local']}")
    print(f"  → From Pit Mac:    {endpoints['mdns']}  (mDNS auto-discovery)")
    print(f"  → Pitlane IP:      {endpoints['lan']}  (Direct Wi-Fi / Ethernet)")
    if extra_lines:
        for line in extra_lines:
            print(f"  {line}")
    print("=" * width + "\n")
    print("Press Ctrl+C to stop the server.\n")

