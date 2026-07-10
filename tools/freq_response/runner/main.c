#include "vfr_fake_subjects.h"
#include "vfr_fof_adapter.h"
#include "vfr_point_measure.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

typedef struct {
    const char* subject_name;
    const char* module_name;
    const char* mode_name;
    const char* freq_file_path;
    const char* out_path;
    VFR_PointMeasureConfig config;
    double fof_b0;
    double fof_b1;
    double fof_a1;
    int has_sample_rate_hz;
    int has_input_amplitude;
    int has_warmup_cycles;
    int has_measure_cycles;
    int has_min_samples_per_cycle;
    int has_max_samples_per_point;
    int has_output_abs_limit;
    int has_gain_floor;
    int has_fof_b0;
    int has_fof_b1;
    int has_fof_a1;
} VFR_RunnerArgs;

static void vfr_print_usage(FILE* stream) {
    fprintf(stream,
            "Usage: voloop_freq_response_runner "
            "--module fake --mode NAME --sample-rate-hz HZ --input-amplitude AMP "
            "--warmup-cycles N --measure-cycles N --min-samples-per-cycle N "
            "--max-samples-per-point N --output-abs-limit LIMIT --gain-floor FLOOR "
            "--freq-file PATH [--out PATH]\n"
            "FOF discrete: --module fof --mode discrete --fof-b0 B0 --fof-b1 B1 --fof-a1 A1\n"
            "Legacy: --subject NAME is temporarily supported for fake modes.\n");
}

static int vfr_parse_double(const char* text, double* value) {
    char* end = NULL;
    double parsed;

    if (text == NULL || value == NULL) {
        return 0;
    }

    errno = 0;
    parsed = strtod(text, &end);
    if (errno != 0 || end == text || *end != '\0' || !isfinite(parsed)) {
        return 0;
    }

    *value = parsed;
    return 1;
}

static int vfr_parse_uint32(const char* text, uint32_t* value) {
    char* end = NULL;
    unsigned long parsed;

    if (text == NULL || value == NULL) {
        return 0;
    }
    if (text[0] == '-') {
        return 0;
    }

    errno = 0;
    parsed = strtoul(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || parsed > 4294967295UL) {
        return 0;
    }

    *value = (uint32_t)parsed;
    return 1;
}

static int vfr_take_value(int argc, char** argv, int* index, const char** value) {
    if (*index + 1 >= argc) {
        return 0;
    }
    *value = argv[*index + 1];
    *index += 1;
    return 1;
}

static int vfr_parse_args(int argc, char** argv, VFR_RunnerArgs* args) {
    int i;

    if (args == NULL) {
        return 0;
    }

    *args = (VFR_RunnerArgs){0};

    for (i = 1; i < argc; ++i) {
        const char* value = NULL;

        if (strcmp(argv[i], "--subject") == 0) {
            if (!vfr_take_value(argc, argv, &i, &args->subject_name)) {
                return 0;
            }
        } else if (strcmp(argv[i], "--module") == 0) {
            if (!vfr_take_value(argc, argv, &i, &args->module_name)) {
                return 0;
            }
        } else if (strcmp(argv[i], "--mode") == 0) {
            if (!vfr_take_value(argc, argv, &i, &args->mode_name)) {
                return 0;
            }
        } else if (strcmp(argv[i], "--sample-rate-hz") == 0) {
            if (!vfr_take_value(argc, argv, &i, &value) ||
                !vfr_parse_double(value, &args->config.sample_rate_hz)) {
                return 0;
            }
            args->has_sample_rate_hz = 1;
        } else if (strcmp(argv[i], "--input-amplitude") == 0) {
            if (!vfr_take_value(argc, argv, &i, &value) ||
                !vfr_parse_double(value, &args->config.input_amplitude)) {
                return 0;
            }
            args->has_input_amplitude = 1;
        } else if (strcmp(argv[i], "--warmup-cycles") == 0) {
            if (!vfr_take_value(argc, argv, &i, &value) ||
                !vfr_parse_uint32(value, &args->config.warmup_cycles)) {
                return 0;
            }
            args->has_warmup_cycles = 1;
        } else if (strcmp(argv[i], "--measure-cycles") == 0) {
            if (!vfr_take_value(argc, argv, &i, &value) ||
                !vfr_parse_uint32(value, &args->config.measure_cycles)) {
                return 0;
            }
            args->has_measure_cycles = 1;
        } else if (strcmp(argv[i], "--min-samples-per-cycle") == 0) {
            if (!vfr_take_value(argc, argv, &i, &value) ||
                !vfr_parse_uint32(value, &args->config.min_samples_per_cycle)) {
                return 0;
            }
            args->has_min_samples_per_cycle = 1;
        } else if (strcmp(argv[i], "--max-samples-per-point") == 0) {
            if (!vfr_take_value(argc, argv, &i, &value) ||
                !vfr_parse_uint32(value, &args->config.max_samples_per_point)) {
                return 0;
            }
            args->has_max_samples_per_point = 1;
        } else if (strcmp(argv[i], "--output-abs-limit") == 0) {
            if (!vfr_take_value(argc, argv, &i, &value) ||
                !vfr_parse_double(value, &args->config.output_abs_limit)) {
                return 0;
            }
            args->has_output_abs_limit = 1;
        } else if (strcmp(argv[i], "--gain-floor") == 0) {
            if (!vfr_take_value(argc, argv, &i, &value) ||
                !vfr_parse_double(value, &args->config.gain_floor)) {
                return 0;
            }
            args->has_gain_floor = 1;
        } else if (strcmp(argv[i], "--fof-b0") == 0) {
            if (!vfr_take_value(argc, argv, &i, &value) ||
                !vfr_parse_double(value, &args->fof_b0)) {
                return 0;
            }
            args->has_fof_b0 = 1;
        } else if (strcmp(argv[i], "--fof-b1") == 0) {
            if (!vfr_take_value(argc, argv, &i, &value) ||
                !vfr_parse_double(value, &args->fof_b1)) {
                return 0;
            }
            args->has_fof_b1 = 1;
        } else if (strcmp(argv[i], "--fof-a1") == 0) {
            if (!vfr_take_value(argc, argv, &i, &value) ||
                !vfr_parse_double(value, &args->fof_a1)) {
                return 0;
            }
            args->has_fof_a1 = 1;
        } else if (strcmp(argv[i], "--freq-file") == 0) {
            if (!vfr_take_value(argc, argv, &i, &args->freq_file_path)) {
                return 0;
            }
        } else if (strcmp(argv[i], "--out") == 0) {
            if (!vfr_take_value(argc, argv, &i, &args->out_path)) {
                return 0;
            }
        } else {
            return 0;
        }
    }

    if (args->subject_name != NULL && (args->module_name != NULL || args->mode_name != NULL)) {
        fprintf(stderr, "legacy --subject cannot be combined with --module or --mode\n");
        return 0;
    }
    if (args->subject_name != NULL) {
        args->module_name = "fake";
        args->mode_name = args->subject_name;
    } else if ((args->module_name == NULL) != (args->mode_name == NULL)) {
        fprintf(stderr, "--module and --mode must be provided together\n");
        return 0;
    }

    return args->module_name != NULL && args->mode_name != NULL && args->freq_file_path != NULL &&
           args->has_sample_rate_hz && args->has_input_amplitude &&
           args->has_warmup_cycles && args->has_measure_cycles &&
           args->has_min_samples_per_cycle && args->has_max_samples_per_point &&
           args->has_output_abs_limit && args->has_gain_floor;
}

static int vfr_validate_global_config(const VFR_PointMeasureConfig* config) {
    if (config == NULL) {
        return 0;
    }

    return isfinite(config->sample_rate_hz) && config->sample_rate_hz > 0.0 &&
           isfinite(config->input_amplitude) && config->input_amplitude > 0.0 &&
           isfinite(config->output_abs_limit) && config->output_abs_limit > 0.0 &&
           isfinite(config->gain_floor) && config->gain_floor > 0.0 &&
           config->measure_cycles > 0U &&
           config->min_samples_per_cycle > 0U &&
           config->max_samples_per_point > 0U;
}

static void vfr_write_csv_header(FILE* out) {
    fprintf(out,
            "module,mode,sample_rate_hz,frequency_hz,input_amplitude,output_amplitude,"
            "gain_linear,gain_db,phase_deg,warmup_samples,measure_samples,total_samples,status\n");
}

static void vfr_write_csv_row(FILE* out,
                              const char* module_name,
                              const char* mode_name,
                              const VFR_PointMeasureConfig* config,
                              const VFR_PointMeasureResult* result) {
    fprintf(out,
            "%s,%s,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%u,%u,%u,%s\n",
            module_name, mode_name, config->sample_rate_hz, result->frequency_hz,
            result->input_amplitude, result->output_amplitude, result->gain_linear,
            result->gain_db, result->phase_deg, result->warmup_samples,
            result->measure_samples, result->total_samples,
            VFR_PointMeasureStatusToString(result->status));
}

static void vfr_write_bad_frequency_row(FILE* out,
                                        const char* module_name,
                                        const char* mode_name,
                                        const VFR_PointMeasureConfig* config) {
    fprintf(out,
            "%s,%s,%.17g,nan,%.17g,nan,nan,nan,nan,0,0,0,%s\n",
            module_name, mode_name, config->sample_rate_hz, config->input_amplitude,
            VFR_PointMeasureStatusToString(VFR_POINT_INVALID_FREQUENCY));
}

int main(int argc, char** argv) {
    VFR_RunnerArgs args;
    VFR_TestSubject subject;
    VFR_FofSubject fof_subject;
    FILE* freq_file;
    FILE* out;
    char line[256];

    if (!vfr_parse_args(argc, argv, &args)) {
        vfr_print_usage(stderr);
        return 2;
    }
    if (!vfr_validate_global_config(&args.config)) {
        fprintf(stderr, "Invalid global measurement configuration\n");
        return 2;
    }

    if (strcmp(args.module_name, "fake") == 0) {
        if (!VFR_GetFakeSubjectByMode(args.mode_name, &subject)) {
            fprintf(stderr, "unsupported fake mode: %s\n", args.mode_name);
            return 2;
        }
    } else if (strcmp(args.module_name, "fof") == 0) {
        if (strcmp(args.mode_name, "discrete") != 0) {
            fprintf(stderr, "unsupported fof mode: %s\n", args.mode_name);
            return 2;
        }
        if (!args.has_fof_b0 || !args.has_fof_b1 || !args.has_fof_a1) {
            fprintf(stderr,
                    "fof discrete requires --fof-b0, --fof-b1, and --fof-a1\n");
            return 2;
        }
        if (!VFR_InitFofDiscreteSubject(&fof_subject, &subject, (float)args.fof_b0,
                                        (float)args.fof_b1, (float)args.fof_a1)) {
            fprintf(stderr, "failed to initialize fof discrete subject\n");
            return 2;
        }
    } else {
        fprintf(stderr, "unsupported module: %s\n", args.module_name);
        fprintf(stderr, "currently supported modules: fake, fof\n");
        return 2;
    }

    freq_file = fopen(args.freq_file_path, "r");
    if (freq_file == NULL) {
        fprintf(stderr, "Failed to open frequency file: %s\n", args.freq_file_path);
        return 2;
    }

    out = stdout;
    if (args.out_path != NULL) {
        out = fopen(args.out_path, "w");
        if (out == NULL) {
            fprintf(stderr, "Failed to create output file: %s\n", args.out_path);
            fclose(freq_file);
            return 2;
        }
    }

    vfr_write_csv_header(out);

    while (fgets(line, sizeof(line), freq_file) != NULL) {
        char* end = NULL;
        double frequency_hz;
        VFR_PointMeasureResult result = {0};
        VFR_PointMeasureStatus status;

        errno = 0;
        frequency_hz = strtod(line, &end);
        while (end != NULL && (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n')) {
            ++end;
        }
        if (end == line || errno != 0 || end == NULL || *end != '\0' || !isfinite(frequency_hz)) {
            char* cursor = line;
            while (*cursor == ' ' || *cursor == '\t' || *cursor == '\r' || *cursor == '\n') {
                ++cursor;
            }
            if (*cursor == '\0') {
                continue;
            }
            args.config.frequency_hz = 0.0;
            vfr_write_bad_frequency_row(out, args.module_name, args.mode_name, &args.config);
            continue;
        }

        args.config.frequency_hz = frequency_hz;
        status = VFR_MeasurePoint(&subject, &args.config, &result);
        result.status = status;
        vfr_write_csv_row(out, args.module_name, args.mode_name, &args.config, &result);
    }

    if (args.out_path != NULL) {
        fclose(out);
    }
    fclose(freq_file);
    return 0;
}
