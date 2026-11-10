# 只补齐Hardware/Host/chassis.py使用的串口占用检查工具，不改控制/规划算法。
FROM slicer-fkie:latest
USER root
RUN apt-get -o Dir::Etc::sourcelist=/etc/apt/sources.list -o Dir::Etc::sourceparts=- update \
 && apt-get install -y --no-install-recommends psmisc \
 && command -v fuser \
 && rm -rf /var/lib/apt/lists/*
