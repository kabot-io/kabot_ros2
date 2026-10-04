import argparse
from pathlib import Path
import socket
import sys


def main():
    parser = argparse.ArgumentParser(description="Claim or release a Zephyr robot or local simulator.")
    parser.add_argument("--host", default="127.0.0.1", help="Robot address (default: local simulator)")
    parser.add_argument("--release", action="store_true")
    arguments = parser.parse_args()
    address = socket.gethostbyname(arguments.host)
    workspace = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(workspace / "scripts/kabot_io"))
    from proto_codec import decode_bonjour_response, encode_bonjour

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as discovery:
        discovery.settimeout(2)
        discovery.sendto(
            encode_bonjour(30011, claim=not arguments.release, release=arguments.release),
            (address, 30012),
        )
        payload, sender = discovery.recvfrom(1024)
    if sender[0] != address:
        raise RuntimeError(f"Unexpected claim responder: {sender}")
    response = decode_bonjour_response(payload)
    if response.is_claimed != (not arguments.release):
        raise RuntimeError("Robot did not accept the requested claim state")
    print(f"claimed={response.is_claimed}")


if __name__ == "__main__":
    main()