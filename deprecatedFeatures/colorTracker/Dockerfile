FROM alpine:3.9 as builder
MAINTAINER Christian Berger christian.berger@gu.se
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
    make && make test && cp colortracker /tmp

# Deploy.
FROM alpine:3.9
MAINTAINER Christian Berger christian.berger@gu.se
RUN apk update && \
    apk --no-cache add \
        libstdc++
WORKDIR /opt
COPY --from=builder /tmp/colortracker .
CMD ["/opt/colortracker"]
