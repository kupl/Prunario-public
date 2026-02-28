FROM ghcr.io/autowarefoundation/autoware:universe-devel-cuda-0.39.1-amd64

RUN apt-get update
RUN apt-get install -y wget unzip tmux inetutils-ping curl vim git llvm-dev libclang-dev cmake
RUN printf 'set-option -g default-shell /bin/bash\nset -g default-terminal "tmux-256color"' > /etc/tmux.conf

CMD bash