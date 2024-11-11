// Copyright (c) 2023 by Rockchip Electronics Co., Ltd. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*-------------------------------------------
                Includes
-------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <chrono>
#include <iostream>
#include "ppseg.h"
#include "image_utils.h"
#include "file_utils.h"

// #define IMAGENET_CLASSES_FILE "./model/synset.txt"

/*-------------------------------------------
                  Main Function
-------------------------------------------*/
int main(int argc, char** argv)
{
    if (argc != 3) {
        printf("%s <model_path> <image_path>\n", argv[0]);
        return -1;
    }

    const char* model_path = argv[1];
    const char* image_path = argv[2];

    int ret;
    rknn_app_context_t rknn_app_ctx;
    memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));

    ret = init_ppseg_model(model_path, &rknn_app_ctx);
    if (ret != 0) {
        printf("init_ppseg_model fail! ret=%d model_path=%s\n", ret, model_path);
        return -1;
    }

    image_buffer_t src_image;
    memset(&src_image, 0, sizeof(image_buffer_t));
    ret = read_image(image_path, &src_image);
    if (ret != 0) {
        printf("read image fail! ret=%d image_path=%s\n", ret, image_path);
        return -1;
    }
    
    image_buffer_t result_image;
    memset(&result_image, 0, sizeof(image_buffer_t));
    result_image.height = rknn_app_ctx.model_height;
    result_image.width = rknn_app_ctx.model_width;
    result_image.format = IMAGE_FORMAT_RGB888;

    ret = inference_ppseg_model(&rknn_app_ctx, &src_image, &result_image);
    if (ret != 0) {
        printf("inference_ppseg_model fail! ret=%d\n", ret);
        goto out;
    }

    ret = convert_image(&result_image, &src_image, NULL, NULL, 0);
    if (ret < 0) {
        printf("convert_image fail! ret=%d\n", ret);
        return -1;
    }

    ret = write_image("./result.png",&src_image);

out:
    ret = release_ppseg_model(&rknn_app_ctx);
    if (ret != 0) {
        printf("release_ppseg_model fail! ret=%d\n", ret);
    }

    if (src_image.virt_addr != NULL) {
        free(src_image.virt_addr);
    }

    if (result_image.virt_addr != NULL) {
        free(result_image.virt_addr);
    }

    return 0;
}
