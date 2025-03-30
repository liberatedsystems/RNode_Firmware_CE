ARG FROM=ubuntu:latest
FROM ${FROM}
ENV PATH="~/.local/bin:${PATH}"
RUN apt update && \
  DEBIAN_FRONTEND=noninteractive apt --yes upgrade && \
  DEBIAN_FRONTEND=noninteractive apt --yes install bzip2 curl git make python3 python3-pip && \
  rm -rf /var/lib/apt/lists/* && \
  update-alternatives --install /usr/bin/python python /usr/bin/python3 1 && \
  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=/usr/local/bin sh -s 1.1.1

# How to build/run this container (should also work with docker CLI):
#   nerdctl build --file Dockerfile --tag rnode-builder:latest .
#   nerdctl run --interactive \
#     --mount src=${PWD},target=/RNode_Firmware_CE,type=bind \
#     --mount src=${HOME},target=/root,type=bind \
#     --tty --rm rnode-builder:latest

# Arduino expects the same name of the directory inside the container as it is named outside, e.g.:  "RNode_Firmware_CE".
#   See https://github.com/microsoft/vscode-arduino/issues/1665#issuecomment-1764234405

# Once there's a release after "1.2.0", we can remove the "-s" option for the installer script.
#   See https://github.com/arduino/arduino-cli/issues/2860
#   See https://arduino.github.io/arduino-cli/latest/installation
