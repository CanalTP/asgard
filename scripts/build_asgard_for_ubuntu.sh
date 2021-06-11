#!/bin/sh


usage() { echo "Usage: $0 -a <path_to_asgard_project>" 1>&2; exit 1; }

while getopts ":a:b:o:" option; do
    case "${option}" in
        a)
            asgard_dir=${OPTARG}
            ;;
    esac
done

shift $((OPTIND-1))
if [ -z "${asgard_dir}" ]; then
    echo "Bad parameters. Doing Nothing."
    usage
    exit 1
fi


echo "** launch dev env installation"

echo "** apt update"
sudo apt -y update

echo "** install dependencies pkg"
sudo apt install -y \
      git \
      autoconf \
      automake \
      make \
      libtool \
      pkg-config \
      g++ \
      gcc \
      locales \
      protobuf-compiler \
      libboost-all-dev \
      libcurl4-openssl-dev \
      libprotobuf-dev \
      libgeos-dev \
      libgeos++-dev \
      libluajit-5.1-dev \
      luajit \
      libspatialite-dev \
      libsqlite3-dev \
      spatialite-bin \
      liblz4-dev \
      unzip \
      python-all-dev \
      vim-common \
      jq \
      gucharmap \
      libprotobuf10 \
      libsqlite3-0 \
      libboost-all-dev \
      libboost-date-time1.62.0 \
      libboost-filesystem1.62.0 \
      libboost-program-options1.62.0 \
      libboost-regex1.62.0 \
      libboost-system1.62.0 \
      libboost-thread1.62.0 \
      libboost-iostreams1.62.0 \
      cmake \
      libzmq3-dev \
      ca-certificates \
      clang-format-4.0 \
      python3 \
      python3-pip


# Install pre-commit
echo "** install pre-commit (pip)"
pip3 install pre-commit


# Clone Valhalla
echo "** clone Valhalla"
cd ${asgard_dir}
git clone https://github.com/valhalla/valhalla.git libvalhalla
cd libvalhalla
git submodule update --init --recursive --depth=1
git checkout 3.1.2
cd -

# Build and install Valhalla
echo "** build and install Valhalla"
valhalla_install_dir=${asgard_dir}/valhalla_install
mkdir -p ${valhalla_install_dir}
mkdir -p libvalhalla/build
cd libvalhalla/build
cmake .. -DCMAKE_BUILD_TYPE=Release -DENABLE_SERVICES=Off -DENABLE_PYTHON_BINDINGS=Off -DENABLE_BENCHMARKS=Off -DENABLE_THREAD_SAFE_TILE_REF_COUNT=On -DCMAKE_INSTALL_PREFIX:PATH=${valhalla_install_dir}
make -j$(nproc) install


# Build Asgard
echo "** build and install Asgard"
cd ${asgard_dir}
git submodule update --init --recursive
mkdir -p build && cd build
cmake .. -DVALHALLA_INCLUDEDIR=${valhalla_install_dir}/include -DVALHALLA_LIBRARYDIR=${valhalla_install_dir}/lib -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

