export VER=1.0

BUILDNR=$(cat packageNumber)
DRIVER_NAME="amc-pico-driver"

echo $(echo $BUILDNR+1 | bc) > packageNumber
BUILDNR=$(echo $BUILDNR+1 | bc)

echo "Making " /usr/src/$DRIVER_NAME-$VER.$BUILDNR
mkdir -p  /usr/src/$DRIVER_NAME-$VER.$BUILDNR

cp -r * /usr/src/$DRIVER_NAME-$VER.$BUILDNR
rm -rf /usr/src/$DRIVER_NAME-$VER.$BUILDNR/docs

dkms build -m $DRIVER_NAME/$VER.$BUILDNR

dkms mkdsc -m $DRIVER_NAME/$VER.$BUILDNR --source-only

dkms mkdeb -m $DRIVER_NAME/$VER.$BUILDNR --source-only
