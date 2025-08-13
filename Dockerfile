FROM ubuntu:24.04

RUN apt update && apt install -y \
    python3-dev \
    python3-pip \
    python3-venv
