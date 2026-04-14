docker run --name qpicard_depth --entrypoint bash -it --runtime=nvidia --gpus all --rm --network=host \
    --env="DISPLAY=$DISPLAY" \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    -v ~/Offroad-Nav/:/workspace/Offroad-Nav \
    mono_depth:latest