FROM alpine:3.7 as builder
MAINTAINER Elsada Lagumdzic elsada@chalmers.se
RUN apk update && \
    apk --no-cache add \
        ca-certificates \
        cmake \
        g++ \
        make
ADD . /opt/sources
WORKDIR /opt/sources
RUN cd /opt/sources && \
    mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release .. && \
    make inputDirection && cp inputDirection /tmp

# Deploy.
FROM alpine:3.7
MAINTAINER Elsada Lagumdzic elsada@chalmers.se
RUN apk update && \
    apk --no-cache add \
        libstdc++ libgcc && \
    mkdir /opt
WORKDIR /opt
COPY --from=builder /tmp/inputDirection .
CMD ["/bin/sh","-c","/opt/inputDirection --cid=112 --verbose"]
