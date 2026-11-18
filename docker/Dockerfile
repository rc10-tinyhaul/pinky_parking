# pinky_parking 시뮬레이션 환경: Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic + Nav2
# 작업공간은 /ws, 이 저장소는 /ws/src/pinky_parking (직접 설치할 때의 표준 구조와 같음)

# 저장소의 package.xml 만 모은다. 코드만 바뀌면 아래 의존성 설치 단계는 캐시를 그대로 쓴다.
FROM osrf/ros:jazzy-desktop AS manifests
COPY . /src
RUN mkdir /manifests && cd /src && find . -name package.xml -exec cp --parents {} /manifests \;

FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1

# 호스트 사용자와 같은 UID/GID 의 일반 사용자로 실행한다 (이미지의 기본 사용자 ubuntu 를 맞춰 쓴다).
# 연결한 저장소에 컨테이너가 만든 파일이 호스트에서 root 소유가 되지 않는다.
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupmod --non-unique --gid $USER_GID ubuntu \
    && usermod --non-unique --uid $USER_UID --gid $USER_GID ubuntu

# 외부 저장소(pinky_pro)를 .repos 에 적힌 고정 버전으로 받는다
COPY pinky_parking.repos /tmp/pinky_parking.repos
RUN mkdir -p /ws/src && cd /ws/src && vcs import --input /tmp/pinky_parking.repos

# 의존성은 package.xml 기준으로 rosdep 이 설치한다 (pinky_pro 는 시뮬레이션에 필요한 패키지만)
COPY --from=manifests /manifests /ws/src/pinky_parking
RUN apt-get update && rosdep update --rosdistro jazzy \
    && rosdep install -y --rosdistro jazzy --ignore-src --from-paths \
         /ws/src/pinky_pro/pinky_description /ws/src/pinky_pro/pinky_gz_sim \
         /ws/src/pinky_pro/pinky_navigation /ws/src/pinky_parking \
    && apt-get install -y --no-install-recommends sudo \
    && echo 'ubuntu ALL=(root) NOPASSWD:ALL' > /etc/sudoers.d/ubuntu \
    && rm -rf /var/lib/apt/lists/* \
    && chown -R ubuntu:ubuntu /ws

# 이 저장소의 모든 패키지를 빌드한다
COPY --chown=ubuntu:ubuntu . /ws/src/pinky_parking
USER ubuntu
RUN . /opt/ros/jazzy/setup.sh && cd /ws \
    && colcon build --symlink-install --event-handlers console_direct- \
         --packages-up-to $(colcon list --base-paths src/pinky_parking --names-only) \
    && rm -rf log \
    && echo 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash' >> /home/ubuntu/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
WORKDIR /ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
