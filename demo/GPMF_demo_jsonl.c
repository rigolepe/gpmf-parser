/*! @file GPMF_demo_jsonl.c
 *
 *  @brief Demo to extract GPMF from an MP4 and output as JSONL
 *
 *  @version 1.0.0
 *
 *  Based on GPMF_demo.c from GoPro Inc.
 *  Extended with JSONL output and sensor exclusion support
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <locale.h>
#include <math.h>

#include "../GPMF_parser.h"
#include "GPMF_mp4reader.h"
#include "../GPMF_utils.h"

#define MAX_EXCLUSIONS 32

// Exclusion list structure
typedef struct {
    uint32_t fourcc[MAX_EXCLUSIONS];
    int count;
} ExclusionList;

// Global variables
uint32_t output_jsonl = 0;
ExclusionList exclusions = {0};
FILE* jsonl_output = NULL;

// Function prototypes
void printHelp(char* name);
int is_excluded(uint32_t fourcc);
void parse_exclusion_list(const char* list);
void outputPayloadJSONL(GPMF_stream* ms, uint32_t payload_id, const char* filename,
                       double start_time, double end_time);
void output_sensor_json(GPMF_stream* ms, uint32_t fourcc, int first_sensor);

void printHelp(char* name)
{
    printf("usage: %s <file_with_GPMF> <optional features>\n", name);
    printf("       -j            Output as JSONL (one payload per line)\n");
    printf("       -o <file>     Output JSONL to file (default: stdout)\n");
    printf("       -e<FOURCC>    Exclude sensor (e.g., -eSCEN -eFACE)\n");
    printf("       -E<list>      Exclude multiple sensors comma-separated (-ESCEN,FACE,MWET)\n");
    printf("       -h            This help\n");
    printf("       \n");
    printf("       JSONL output format: One JSON object per payload (~1 second)\n");
    printf("       ver 1.0\n");
}

int is_excluded(uint32_t fourcc)
{
    for (int i = 0; i < exclusions.count; i++) {
        if (exclusions.fourcc[i] == fourcc) return 1;
    }
    return 0;
}

void parse_exclusion_list(const char* list)
{
    char buffer[256];
    strncpy(buffer, list, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char* token = strtok(buffer, ",");
    while (token != NULL && exclusions.count < MAX_EXCLUSIONS) {
        if (strlen(token) == 4) {
            // Build FOURCC manually to avoid macro issues
            exclusions.fourcc[exclusions.count++] =
                ((token[0] << 0) | (token[1] << 8) | (token[2] << 16) | (token[3] << 24));
        }
        token = strtok(NULL, ",");
    }
}

void output_json_array(double* data, uint32_t elements, uint32_t samples)
{
    fprintf(jsonl_output, "[");
    for (uint32_t i = 0; i < samples; i++) {
        if (i > 0) fprintf(jsonl_output, ",");

        if (elements == 1) {
            fprintf(jsonl_output, "%.6f", data[i]);
        } else if (elements == 3) {
            fprintf(jsonl_output, "[%.6f,%.6f,%.6f]",
                   data[i*3], data[i*3+1], data[i*3+2]);
        } else if (elements == 4) {
            fprintf(jsonl_output, "[%.6f,%.6f,%.6f,%.6f]",
                   data[i*4], data[i*4+1], data[i*4+2], data[i*4+3]);
        } else if (elements == 9) {
            // GPS9 special case
            fprintf(jsonl_output, "[%.10f,%.10f,%.3f,%.3f,%.3f,%.2f,%d,%d,%d]",
                   data[i*9], data[i*9+1], data[i*9+2], data[i*9+3], data[i*9+4],
                   data[i*9+5], (int)data[i*9+6], (int)data[i*9+7], (int)data[i*9+8]);
        } else {
            // Generic case
            fprintf(jsonl_output, "[");
            for (uint32_t j = 0; j < elements; j++) {
                if (j > 0) fprintf(jsonl_output, ",");
                fprintf(jsonl_output, "%.6f", data[i*elements + j]);
            }
            fprintf(jsonl_output, "]");
        }
    }
    fprintf(jsonl_output, "]");
}

void output_sensor_json(GPMF_stream* ms, uint32_t fourcc, int first_sensor)
{
    if (is_excluded(fourcc)) return;

    // Reset to start of stream
    GPMF_ResetState(ms);

    // Find the sensor
    if (GPMF_OK != GPMF_FindNext(ms, fourcc, GPMF_RECURSE_LEVELS)) {
        return;
    }

    uint32_t samples = GPMF_Repeat(ms);
    uint32_t elements = GPMF_ElementsInStruct(ms);

    if (samples == 0) return;

    // Allocate buffer for scaled data
    uint32_t buffersize = samples * elements * sizeof(double);
    double* buffer = (double*)malloc(buffersize);

    if (!buffer) return;

    // Get scaled data
    if (GPMF_OK == GPMF_ScaledData(ms, buffer, buffersize, 0, samples, GPMF_TYPE_DOUBLE)) {
        // Output JSON
        if (!first_sensor) fprintf(jsonl_output, ",");

        char fourcc_str[5] = {0};
        fourcc_str[0] = (fourcc >> 0) & 0xFF;   // First byte
        fourcc_str[1] = (fourcc >> 8) & 0xFF;   // Second byte
        fourcc_str[2] = (fourcc >> 16) & 0xFF;  // Third byte
        fourcc_str[3] = (fourcc >> 24) & 0xFF;  // Fourth byte

        // Convert to lowercase for JSON key
        for (int i = 0; i < 4; i++) {
            if (fourcc_str[i] >= 'A' && fourcc_str[i] <= 'Z') {
                fourcc_str[i] = fourcc_str[i] + 32;
            }
        }

        fprintf(jsonl_output, "\"%s\":{\"count\":%u,\"data\":", fourcc_str, samples);
        output_json_array(buffer, elements, samples);
        fprintf(jsonl_output, "}");
    }

    free(buffer);
}

void outputPayloadJSONL(GPMF_stream* ms, uint32_t payload_id, const char* filename,
                       double start_time, double end_time)
{
    // Start JSON object
    fprintf(jsonl_output, "{");

    // Metadata
    fprintf(jsonl_output, "\"payload_id\":%u,", payload_id);
    fprintf(jsonl_output, "\"filename\":\"%s\",", filename);
    fprintf(jsonl_output, "\"start_time\":%.10f,", start_time);
    fprintf(jsonl_output, "\"end_time\":%.10f,", end_time);
    fprintf(jsonl_output, "\"duration\":%.10f", end_time - start_time);

    // Calculate frame numbers (assuming 24 fps)
    uint32_t frame_start = (uint32_t)(start_time * 24.0);
    uint32_t frame_end = (uint32_t)(end_time * 24.0);
    fprintf(jsonl_output, ",\"frame_start\":%u", frame_start);
    fprintf(jsonl_output, ",\"frame_end\":%u", frame_end);

    // Important sensors in order of priority
    uint32_t sensors[] = {
        STR2FOURCC("ACCL"), // Accelerometer
        STR2FOURCC("GYRO"), // Gyroscope
        STR2FOURCC("GPS9"), // GPS
        STR2FOURCC("SHUT"), // Shutter
        STR2FOURCC("ISOE"), // ISO
        STR2FOURCC("CORI"), // Camera Orientation
        STR2FOURCC("IORI"), // Image Orientation
        STR2FOURCC("GRAV"), // Gravity
        STR2FOURCC("MSKP"), // Media Skip
        STR2FOURCC("LSKP"), // Lapse Skip
        STR2FOURCC("CSCM"), // Color Space/Matrix
        STR2FOURCC("FACE"), // Face Detection
        STR2FOURCC("HUES"), // Hue Statistics
        STR2FOURCC("YAVG"), // Luminance Average
        STR2FOURCC("UNIF"), // Scene Uniformity
        STR2FOURCC("WBAL"), // White Balance
        STR2FOURCC("WRGB"), // White Balance RGB
        STR2FOURCC("MWET"), // Motion/Wind
        STR2FOURCC("WNDM"), // Wind Mode
        STR2FOURCC("AALP"), // Auto Low Light
        STR2FOURCC("SCEN"), // Scene Classification
    };

    int sensor_count = sizeof(sensors) / sizeof(sensors[0]);

    // Output each sensor if not excluded
    for (int i = 0; i < sensor_count; i++) {
        if (!is_excluded(sensors[i])) {
            output_sensor_json(ms, sensors[i], 0);
        }
    }

    // Close JSON object and add newline
    fprintf(jsonl_output, "}\n");
    fflush(jsonl_output);
}

int main(int argc, char* argv[])
{
    GPMF_ERR ret = GPMF_OK;

    // Set UTF-8 locale
    setlocale(LC_ALL, "C.UTF-8");

    if (argc < 2) {
        printHelp(argv[0]);
        return -1;
    }

    // Default output to stdout
    jsonl_output = stdout;

    // Parse arguments
    for (int i = 2; i < argc; i++) {
        if (argv[i][0] == '-') {
            switch (argv[i][1]) {
                case 'j':
                    output_jsonl = 1;
                    break;
                case 'o':
                    if (i + 1 < argc) {
                        jsonl_output = fopen(argv[++i], "w");
                        if (!jsonl_output) {
                            fprintf(stderr, "Error: Cannot open output file %s\n", argv[i]);
                            return -1;
                        }
                    }
                    break;
                case 'e':
                    {
                        char *fourcc_str = &argv[i][2];
                        if (strlen(fourcc_str) == 4) {
                            if (exclusions.count < MAX_EXCLUSIONS) {
                                // Build FOURCC manually to avoid macro issues
                                exclusions.fourcc[exclusions.count++] =
                                    ((fourcc_str[0] << 0) | (fourcc_str[1] << 8) |
                                     (fourcc_str[2] << 16) | (fourcc_str[3] << 24));
                            }
                        }
                    }
                    break;
                case 'E':
                    parse_exclusion_list(&argv[i][2]);
                    break;
                case 'h':
                    printHelp(argv[0]);
                    return 0;
                default:
                    break;
            }
        }
    }

    // Open MP4
    size_t mp4handle = OpenMP4Source(argv[1], MOV_GPMF_TRAK_TYPE, MOV_GPMF_TRAK_SUBTYPE, 0);
    if (mp4handle == 0) {
        fprintf(stderr, "Error: %s is an invalid MP4/MOV or it has no GPMF data\n", argv[1]);
        return -1;
    }

    // Get metadata info
    double metadatalength = GetDuration(mp4handle);
    if (metadatalength > 0.0) {
        uint32_t payloads = GetNumberPayloads(mp4handle);

        // Get base filename without path
        const char* filename = strrchr(argv[1], '/');
        if (filename) {
            filename++;
        } else {
            filename = argv[1];
        }

        // Process each payload
        GPMF_stream metadata_stream = {0};
        size_t payloadres = 0;

        for (uint32_t index = 0; index < payloads; index++) {
            double in = 0.0, out = 0.0;
            uint32_t payloadsize = GetPayloadSize(mp4handle, index);
            payloadres = GetPayloadResource(mp4handle, payloadres, payloadsize);
            uint32_t* payload = GetPayload(mp4handle, payloadres, index);

            if (!payload) continue;

            ret = GetPayloadTime(mp4handle, index, &in, &out);
            if (ret != GPMF_OK) continue;

            ret = GPMF_Init(&metadata_stream, payload, payloadsize);
            if (ret != GPMF_OK) continue;

            if (output_jsonl) {
                outputPayloadJSONL(&metadata_stream, index, filename, in, out);
            } else {
                // Default text output (simplified)
                printf("Payload %d: %.3f - %.3f seconds\n", index, in, out);
            }
        }

        // Free the payload resource
        if (payloadres) {
            FreePayloadResource(mp4handle, payloadres);
        }
    }

    CloseSource(mp4handle);

    if (jsonl_output != stdout && jsonl_output != NULL) {
        fclose(jsonl_output);
    }

    return 0;
}