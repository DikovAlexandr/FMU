FROM ubuntu:22.04

# RUN apt-get update && \
#     apt-get install -y openjdk-17-jdk python3 python3-pip maven && \
#     apt-get clean;

RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    apt-get update && \
    apt-get install -y \
      openjdk-17-jdk \
      python3 python3-pip \
      maven && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Копируем локальные библиотеки и pom.xml
COPY lib/ /workspace/lib/
COPY pom.xml .

# Устанавливаем локальные JAR в Maven репозиторий контейнера
RUN mvn install:install-file -Dfile=lib/fmi-import-0.38.0.jar -DgroupId=no.ntnu.ihb.fmi4j -DartifactId=fmi-import -Dversion=0.38.0 -Dpackaging=jar
RUN mvn install:install-file -Dfile=lib/fmi-md-0.38.0.jar -DgroupId=no.ntnu.ihb.fmi4j.modeldescription -DartifactId=fmi-md -Dversion=0.38.0 -Dpackaging=jar

# Копируем остальной код и собираем проект
COPY src src/
RUN mvn clean package

# Устанавливаем Python зависимости
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt
RUN apt-get update && apt-get install -y python3-dev unzip

# Настраиваем переменные окружения
ENV LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/usr/lib/python3.10/config-3.10-x86_64-linux-gnu:$LD_LIBRARY_PATH
ENV LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libpython3.10.so

CMD ["/bin/bash"]