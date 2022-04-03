FROM ubuntu:18.04

LABEL maintainer="iwatake <take.iwiw2222@gmail.com>"
ENV TZ JST-9
SHELL ["/bin/bash", "-c"]

RUN apt-get update \
&& apt-get install -y python3-pip python3-dev \
&& cd /usr/local/bin \
&& ln -s /usr/bin/python3 python \
&& pip3 install --upgrade pip
ENTRYPOINT ["python3"]
