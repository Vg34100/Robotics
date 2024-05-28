import logging
import socket

def cot_init():
    try:
        # Log starting of the initialization
        logging.info("COT: Initializing CoT UDP socket...")

        # Create a UDP socket for sending CoT messages
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

        # Log the creation of the socket
        logging.info("COT: UDP socket created.")

        # Set socket options
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)

        # Log setting the socket option
        logging.info("COT: Set socket IP_MULTICAST_TTL to 2.")

        # Log successful initialization
        logging.info("COT: CoT UDP socket initialization successful.")

        return sock

    except Exception as e:
        # Log any exceptions or errors during initialization
        logging.error(f"COT: Error initializing CoT UDP socket: {e}")
        return None
