
VER="1.0"
BUILDNR=$(cat packageNumber)
DRIVER_NAME="amc-pico-driver"

make clean

echo "Making " ../build/$DRIVER_NAME-$VER.$BUILDNR
mkdir -p ../build/$DRIVER_NAME-$VER.$BUILDNR

cp -r * ../build/$DRIVER_NAME-$VER.$BUILDNR/

rm -rf ../build/$DRIVER_NAME-$VER.$BUILDNR/.git
rm -rf ../build/$DRIVER_NAME-$VER.$BUILDNR/amc-pico-driver-dkms-mkdeb
rm -rf ../build/$DRIVER_NAME-$VER.$BUILDNR/docs
rm -rf ../build/$DRIVER_NAME-$VER.$BUILDNR/dkms.conf
rm -rf ../build/$DRIVER_NAME-$VER.$BUILDNR/build_dkms.sh
rm -rf ../build/$DRIVER_NAME-$VER.$BUILDNR/create_tar_gz.sh
rm -rf ../build/$DRIVER_NAME-$VER.$BUILDNR/*~

cd ../build && tar czvf $DRIVER_NAME-$VER.$BUILDNR.tar.gz $DRIVER_NAME-$VER.$BUILDNR
