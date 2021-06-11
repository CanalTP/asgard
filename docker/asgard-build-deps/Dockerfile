FROM ubuntu:20.04

# Now, go through and install the build dependencies
RUN apt-get update --assume-yes
RUN env DEBIAN_FRONTEND=noninteractive apt-get install --yes --quiet \
    autoconf \
    automake \
    ccache \
    clang \
    clang-format \
    clang-tidy \
    coreutils \
    curl \
    cmake \
    g++ \
    gcc \
    git \
    jq \
    lcov \
    libboost${boost_version}-all-dev \
    libcurl4-openssl-dev \
    libgeos++-dev \
    libgeos-dev \
    libluajit-5.1-dev \
    liblz4-dev \
    libprotobuf-dev \
    libspatialite-dev \
    libsqlite3-dev \
    libsqlite3-mod-spatialite \
    libtool \
    lld \
    locales \
    luajit \
    make \
    osmium-tool \
    parallel \
    pkg-config \
    protobuf-compiler \
    python-all-dev \
    python3-all-dev \
    python3-minimal \
    spatialite-bin \
    unzip \
    zlib1g-dev \
    libzmq3-dev \
    wget \
    python \
  && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/valhalla/valhalla.git libvalhalla \
  && cd libvalhalla \
  && git submodule update --init --recursive --depth=1 \
  && git checkout 3.1.2 \
  && cd -

RUN mkdir -p libvalhalla/build \
  && cd libvalhalla/build \
  && cmake .. -DCMAKE_BUILD_TYPE=Release -DENABLE_SERVICES=Off -DENABLE_NODE_BINDINGS=Off -DENABLE_PYTHON_BINDINGS=Off \
              -DBUILD_SHARED_LIBS=Off -DBoost_USE_STATIC_LIBS=ON -DENABLE_THREAD_SAFE_TILE_REF_COUNT=On\
  && make -j$(nproc) install && ldconfig

