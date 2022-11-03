#!/vendor/bin/sh

MSIM_DEVICES=(
    xq-bc52 xq-bc72 # Dual-SIM version
)
MSIM_DEVICE=0

for device in "${MSIM_DEVICES[@]}"; do
    if grep -qi "Model: ${device}" /dev/block/bootdevice/by-name/LTALabel; then
        MSIM_DEVICE=1
        model=$device
        break
    fi
done

if [[ "${MSIM_DEVICE}" -eq 1 ]]; then
    setprop vendor.radio.multisim.config dsds
fi

if [[ $model == "xq-bc52" ]]; then
    setprop ro.vendor.product.rf.id PDX215-A2
else
    setprop ro.vendor.product.rf.id PDX215-C2
fi

