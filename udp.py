from socket import *

serverSocket = socket(AF_INET, SOCK_DGRAM)


def connect(serverSocket, port):
    serverSocket.bind(("", 1212 + port))
    return serverSocket


def init(serverSocket, port):
    serverSocket.sendto(bytes("READY", "utf-8"), ("", port))

    for i in range(0, 2):
        message, address = serverSocket.recvfrom(1024)

        print(message)

    rv = {}

    for i in range(10):
        message, address = serverSocket.recvfrom(1024)

        print(i, message)

        message = message.decode("utf-8")

        if message == "MSG_START":
            continue
        if message == "MSG_END":
            break

        tokens = message.split("]")[1].split("\n")

        rv[tokens[0]] = [float(x) for x in tokens[1:-1]]

    return rv


def submit(serverSocket, port, pos):
    print(pos)
    serverSocket.sendto(bytes(pos, "utf-8"), ("", port))


def receive(serverSocket):
    message, address = serverSocket.recvfrom(1024)

    rv = {}

    for i in range(10):
        message, address = serverSocket.recvfrom(1024)

        message = message.decode("utf-8")

        if message == "MSG_START":
            continue
        if message == "MSG_END":
            break

        tokens = message.split("]")[1].split("\n")

        print(message.split("]")[0])

        rv[tokens[0]] = [float(x) for x in tokens[1:-1]]

    return rv
