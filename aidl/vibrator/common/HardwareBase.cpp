/*
 * Copyright (C) 2019 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "HardwareBase.h"

#include <cutils/properties.h>
#include <dlfcn.h>
#include <log/log.h>
#include <cstdint>

#include <fstream>
#include <sstream>

#include "utils.h"

#define LIB_MISCTA "libmiscta.so"

// TA functions
static void* ta_handle = NULL;
static int (*miscta_get_unit_size)(uint32_t unit, uint32_t* size) = NULL;
static int (*miscta_read_unit)(uint32_t id, void* buf, uint32_t* size) = NULL;

namespace aidl {
namespace android {
namespace hardware {
namespace vibrator {

HwApiBase::HwApiBase() {
    mPathPrefix = std::getenv("HWAPI_PATH_PREFIX") ?: "";
    if (mPathPrefix.empty()) {
        ALOGE("Failed get HWAPI path prefix!");
    }
}

void HwApiBase::saveName(const std::string& name, const std::ios* stream) {
    mNames[stream] = name;
}

bool HwApiBase::has(const std::ios& stream) {
    return !!stream;
}

void HwApiBase::debug(int fd) {
    dprintf(fd, "Kernel:\n");

    for (auto& entry : utils::pathsFromEnv("HWAPI_DEBUG_PATHS", mPathPrefix)) {
        auto& path = entry.first;
        auto& stream = entry.second;
        std::string line;

        dprintf(fd, "  %s:\n", path.c_str());
        while (std::getline(stream, line)) {
            dprintf(fd, "    %s\n", line.c_str());
        }
    }

    mRecordsMutex.lock();
    dprintf(fd, "  Records:\n");
    for (auto& r : mRecords) {
        if (r == nullptr) {
            continue;
        }
        dprintf(fd, "    %s\n", r->toString(mNames).c_str());
    }
    mRecordsMutex.unlock();
}

int ta_read_unit(int32_t* arr, int offset, uint32_t unit) {
    uint32_t ta_sz;

    int ret = miscta_get_unit_size(unit, &ta_sz);
    if (ret) {
        ALOGE("%s: Cannot retrieve TA unit %d size error %d", __func__, unit, ret);
        return -1;
    }

    ret = miscta_read_unit(unit, arr + offset, &ta_sz);
    if (ret) {
        ALOGE("%s: Cannot read TA unit %d of size %u: error %d", __func__, unit, ta_sz, ret);
        return -1;
    }

    return ta_sz;
}

int cirrusMiscTaRead(int32_t* a1) {
    uint32_t ta_sz;
    uint32_t unit = 0;

    if ((ta_sz = ta_read_unit(a1, 0, 4730)) < 0) {
        return 1;
    }
    ALOGI("%s: unit = %d, size = %d, val = %d", __func__, unit, ta_sz, a1[0]);

    if ((ta_sz = ta_read_unit(a1, 2, 4731)) < 0) {
        return 1;
    }
    ALOGI("%s: unit = %d, size = %d, val = %ld", __func__, unit, ta_sz, *(uint64_t*)(a1 + 2));

    if ((ta_sz = ta_read_unit(a1, 4, 4732)) < 0) {
        return 1;
    }
    ALOGI("%s: unit = %d, size = %d, val = 0x%x", __func__, unit, ta_sz, a1[4]);

    if ((ta_sz = ta_read_unit(a1, 5, 4733)) < 0) {
        return 1;
    }
    ALOGI("%s: unit = %d, size = %d, val = 0x%x", __func__, unit, ta_sz, a1[5]);

    return 0;
}

HwCalBase::HwCalBase() {
    std::ifstream calfile;
    auto propertyPrefix = std::getenv("PROPERTY_PREFIX");

    if (propertyPrefix != NULL) {
        mPropertyPrefix = std::string(propertyPrefix);
    } else {
        ALOGE("Failed get property prefix!");
    }

    ALOGI("%s: Starting getting vibrator calibration data from TA partition", __func__);
    ta_handle = dlopen(LIB_MISCTA, RTLD_NOW);
    if (ta_handle) {
        // Load related symbol
        miscta_get_unit_size =
                (int (*)(unsigned int, unsigned int*))dlsym(ta_handle, "miscta_get_unit_size");
        if (!miscta_get_unit_size) {
            ALOGE("%s: Cannot find symbol: miscta_get_unit_size", __func__);
            return;
        }

        miscta_read_unit =
                (int (*)(uint32_t, void*, uint32_t*))dlsym(ta_handle, "miscta_read_unit");
        if (!miscta_read_unit) {
            ALOGE("%s: Cannot find symbol: miscta_read_unit", __func__);
            return;
        }

        int32_t dat[6];
        if (cirrusMiscTaRead(dat) == 0) {
            std::string f0_measured = std::to_string(dat[5]);
            std::string redc_measured = std::to_string(dat[4]);

            mCalData[utils::trim("f0_measured")] = utils::trim(f0_measured);
            mCalData[utils::trim("redc_measured")] = utils::trim(redc_measured);
        }
    }
}

void HwCalBase::debug(int fd) {
    std::ifstream stream;
    std::string path;
    std::string line;
    struct context {
        HwCalBase* obj;
        int fd;
    } context{this, fd};

    dprintf(fd, "Properties:\n");

    property_list(
            [](const char* key, const char* value, void* cookie) {
                struct context* context = static_cast<struct context*>(cookie);
                HwCalBase* obj = context->obj;
                int fd = context->fd;
                const std::string expect{obj->mPropertyPrefix};
                const std::string actual{key, std::min(strlen(key), expect.size())};
                if (actual == expect) {
                    dprintf(fd, "  %s:\n", key);
                    dprintf(fd, "    %s\n", value);
                }
            },
            &context);

    dprintf(fd, "\n");

    dprintf(fd, "Persist:\n");

    utils::fileFromEnv("CALIBRATION_FILEPATH", &stream, &path);

    dprintf(fd, "  %s:\n", path.c_str());
    while (std::getline(stream, line)) {
        dprintf(fd, "    %s\n", line.c_str());
    }
}

}  // namespace vibrator
}  // namespace hardware
}  // namespace android
}  // namespace aidl
