from mdb import *
import time


# Main measurement control loop
#
# interacts with the database to pick the next measurements
# and stores the results
#
# runs indefinetly


def m_control(series_names, mfun):
    """
    m_control input:
    seriesNames: list of seriesNames
    mfun: measurement function that takes a (seriesName, configurationId) and returns a map with measurement results
    """
    assert_series(series_names)
    current_series_idx = 0
    current_series = series_names[current_series_idx]
    error_counter = 0
    total_time = 0
    total_count = 0

    while True:
        # change series after 20 errors
        if error_counter > 0 and (error_counter % 20 == 0) and len(series_names) > 1:
            current_series_idx = (current_series_idx + 1) % len(series_names)
            current_series = series_names[current_series_idx]
            print "#switching to series " + current_series

        next_config_id = claim_next_measurement(current_series)

        if next_config_id is None:
            error_counter += 1
            wait = 1
            # slowly increasing waits between errors
            if error_counter > 10:
                wait = 5
            if error_counter > 20:
                wait = 10
            if error_counter > 30:
                wait = 30
            if error_counter > 100:
                wait = 120
            print("#no next measurement found, waiting {0} seconds".format(wait))
            time.sleep(wait)
        else:
            # no error, so let's measure
            error_counter = 0
            t1 = time.time()
            m_result = mfun(current_series, next_config_id)
            if m_result is not None:
                store_measurements(current_series, next_config_id, m_result)
            t2 = time.time()
            total_count += 1
            total_time += (t2 - t1)
            remaining_time = (total_time / total_count) * count_remaining_measurements(series_names)
            print("#analysis time: " + format(t2 - t1, ".2f") + "s, estimated remaining: " + format_time(remaining_time))


def format_time(t):
    d, r = divmod(t, 60 * 60 * 24)
    h, r = divmod(r, 60 * 60)
    m, r = divmod(r, 60)
    return "{0}d {1}h {2}m".format(int(d), int(h), int(m))


def assert_series(series_names):
    # check that series exist
    for s in series_names:
        get_series_id(s)
