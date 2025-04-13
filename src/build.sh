if [ -z "${VERSION}" ]; then
  VERSION=1.0.0
fi

sudo docker build -t carwyn987/robot-arm-slave:$VERSION .
# Should automate (jenkins?) to ensure unique version numbers
#sudo docker push carwyn987/robot-arm-slave:$VERSION