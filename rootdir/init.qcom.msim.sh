#!/vendor/bin/sh

model=`grep -aim1 'model:' /dev/block/bootdevice/by-name/LTALabel | sed -e 's/^.*model:[ ]*\([A-Za-z0-9-]*\).*$/\1/I'` 2> /dev/null

case "$model" in
    "XQ-BC42" | "XQ-BC52" | "XQ-BC62" | "XQ-BC72" | "XQ-BQ42" | "XQ-BQ52" | "XQ-BQ62" | "XQ-BQ72" | "XQ-DQ72")
        setprop vendor.radio.multisim.config dsds;;
    * )
        setprop vendor.radio.multisim.config ss;;
esac

if [ "$model" = "" ]; then
    setprop vendor.radio.ltalabel.model "unknown"
else
    setprop vendor.radio.ltalabel.model "$model"
fi
