#ifndef VFR_FAKE_SUBJECTS_H
#define VFR_FAKE_SUBJECTS_H

/**
 * @file vfr_fake_subjects.h
 * @brief Fake subjects for validating the frequency-response runner.
 */

#include "vfr_test_subject.h"

/**
 * @brief Resolve a fake subject by mode.
 *
 * Supported modes are "unity", "gain2", "negative_unity", "zero", and
 * "exploding".
 *
 * @param mode Fake mode name from the command line.
 * @param subject Output test subject.
 * @return 1 on success, otherwise 0.
 */
int VFR_GetFakeSubjectByMode(const char* mode, VFR_TestSubject* subject);

/**
 * @brief Compatibility wrapper for older fake-mode lookup callers.
 */
int VFR_GetFakeSubject(const char* mode, VFR_TestSubject* subject);

#endif /* VFR_FAKE_SUBJECTS_H */
