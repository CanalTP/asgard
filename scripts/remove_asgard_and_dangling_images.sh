#!/usr/bin/env bash

# ********************************
# Remove Asgard images
# ********************************
echo "Remove Asgard images"
asgard_images=$(docker images "*navitia/asgard-*" -q)
if [ -z "$asgard_images" ]
then
    echo "Asgard images don't exist"
else
    echo "Asgard images ${asgard_images}"
    docker rmi -f ${asgard_images}
fi

# ********************************
# Remove dangling images
# ********************************
echo "Remove dangling images"
dangling_images=$(docker images --filter "dangling=true" -q --no-trunc)
if [ -z "$dangling_images" ]
then
    echo "Dangling images don't exist"
else
    echo "Dangling images ${dangling_images}"
    docker rmi -f ${dangling_images}
fi

echo "Cleaning is finish"
