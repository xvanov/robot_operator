FROM husarnet/husarnet:2.0.180
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
# install common network debug tools
RUN apt-get update && apt-get install -y \
    curl \
    iputils-ping \
    vim \
    iproute2 \
    apt-utils \
    ca-certificates
# install zenoh
RUN echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | tee -a /etc/apt/sources.list > /dev/null && \
    apt-get update -y && \
    apt-get install -y zenoh-bridge-ros2dds || true

# startup with config
COPY ./limit_bandwidth.sh /
COPY ./operator-dds-config.json5 /
COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
