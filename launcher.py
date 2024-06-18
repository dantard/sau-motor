#!/usr/bin/env python3
import argparse
import configparser
import re
import subprocess
import sys
import time

from xdo import Xdo
import yaml
import socket

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='ROS2 latency measurement tool')
    parser.add_argument('-f', '--file', default="data/exp1/exp1.yaml", type=str)
    parser.add_argument('-S', '--setup', action='store_true')
    parser.add_argument('-c', '--ctrl', action='store_true')
    parser.add_argument('-C', '--ctrlexit', action='store_true')
    args = parser.parse_args()
    results = []

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    open_terminals = {}

    with open(args.file, "r") as f:
        data = yaml.load(f, Loader=yaml.FullLoader)

        environments = data["environments"]
        params = data.get("params", {})
        experiment = data["experiments"]["exp1"]
        params.update(experiment.get("params", {}))
        terminals = experiment["terminals"]

        if args.setup:
            port = 12000
            for terminal in terminals:
                env = terminal.get("env")
                run = terminal.get("run")
                setup_commands = environments.get(env, [])
                for i in range(len(run)):
                    for command in setup_commands:
                        if command.startswith("pause"):
                            time.sleep(int(command.split(" ")[1]))
                        else:
                            print(command, port)
                            cmd = command + "\n"
                            sock.sendto(cmd.encode(), ("localhost", port))
                    port += 1

        if args.ctrl or args.ctrlexit:
            count = 0

            for terminal in terminals:
                count += len(terminal.get("run"))

            for i in reversed(range(count)):
                b = chr(3)
                sock.sendto(b.encode(), ("localhost", 12000 + i))
                time.sleep(0.25)
                sock.sendto("\n".encode(), ("localhost", 12000 + i))

            if args.ctrlexit:
                sys.exit(0)

        port = 12000
        for terminal in terminals:
            env = terminal.get("env")
            run = terminal.get("run")

            for i in range(len(run)):
                for k, v in params.items():
                    run[i] = run[i].replace(f"${k}", str(v)) + "\n"
                print(run[i], port)
                sock.sendto(run[i].encode(), ("localhost", port))
                port += 1

        '''

        terminals = data["terminals"]
        environments = data["environments"]

        for i, terminal in enumerate(terminals):
            environment = environments.get(terminal)

            if open_terminals.get(terminal) is None:
                open_terminals[terminal] = []

            open_terminals[terminal].append(i)

            if args.setup:
                if environment is not None:

                    for command in environment:
                        if command.startswith("pause"):
                            time.sleep(int(command.split(" ")[1]))
                        else:
                            cmd = command + "\n"
                            sock.sendto(cmd.encode(), ("localhost", 12000 + i))

                time.sleep(4)

        if args.ctrl or args.ctrlexit:
            terminals = data["terminals"]
            environments = data["environments"]
            for i in reversed(range(len(terminals))):
                b = chr(3)
                sock.sendto(b.encode(), ("localhost", 12000 + i))
                time.sleep(0.25)
                sock.sendto("\n".encode(), ("localhost", 12000 + i))

            if args.ctrlexit:
                sys.exit(0)

        executables = data["experiments"]["exp1"]["nodes"]
        for executable in executables:
            env = executable.get("env")
            run = executable.get("run")
            for i, r in enumerate(run):
                r = r + "\n"
                port = open_terminals[env][i] + 12000
                sock.sendto(r.encode(), ("localhost", port))
        '''
