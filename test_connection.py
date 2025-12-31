import socket
from urllib.parse import urlparse

# Test if we can resolve the domain
url = "https://1a930ddf-5aec-4de1-8165-3aeaffd38684.europe-west3-0.gcp.cloud.qdrant.io"

try:
    parsed = urlparse(url)
    print(f"Testing connection to: {parsed.hostname}")

    # Try to resolve the hostname
    addr_info = socket.getaddrinfo(parsed.hostname, 443)
    print(f"DNS resolution successful: {addr_info[0][4][0]}")

    # Try to create a socket connection
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(5)  # 5 second timeout
        result = sock.connect_ex((parsed.hostname, 443))
        if result == 0:
            print("Connection to port 443 successful")
        else:
            print(f"Connection to port 443 failed with error code: {result}")

except socket.gaierror as e:
    print(f"DNS resolution failed: {e}")
except Exception as e:
    print(f"Connection test failed: {e}")