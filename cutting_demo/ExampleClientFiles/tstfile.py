import psutil
import socket
import os


def get_ipv4_address():
    # Get network interfaces (addresses)
    addrs = psutil.net_if_addrs()

    ips = []
    # Iterate through network interfaces and get IPv4 address
    for interface, interface_addresses in addrs.items():
        for addr in interface_addresses:
            if addr.family == socket.AF_INET:  # Use socket.AF_INET for IPv4
                # return addr.address
                ips.append(addr.address)
                print(f'ip address: {addr.address}')

    for ip in ips:
        if ip[0:10] == '192.170.10':
            return '127.0.0.1'
    for ip in ips:
        if ip[0:10] == '192.168.1.':
            return '192.168.1.35'
    for ip in ips:
        if ip[0:10] == '192.168.0.':
            return '192.168.0.167'
    for ip in ips:
        if ip[0:10] == '192.168.50':
            return '192.168.50.43'
    print(f'unknown ip addres: {ips}')


# Call the function
ipv4_address = get_ipv4_address()
cwd = os.getcwd()
print(f"IPv4 Address: {ipv4_address}")

