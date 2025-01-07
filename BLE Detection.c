#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <time.h>

#define WINDOW_SIZE 10
#define MOVEMENT_THRESHOLD 0.5
#define MANUFACTURER_ID 0xE1

typedef struct {
    int8_t x;
    int8_t y;
    int8_t z;
} AccelData;

typedef struct {
    AccelData buffer[WINDOW_SIZE];
    int buffer_index;
    int buffer_full;
} MotionDetector;

void init_motion_detector(MotionDetector *detector) {
    detector->buffer_index = 0;
    detector->buffer_full = 0;
    memset(detector->buffer, 0, sizeof(AccelData) * WINDOW_SIZE);
}

double calculate_variance(AccelData *data, int count) {
    double sum = 0.0;
    double sum_sq = 0.0;
    
    for (int i = 0; i < count; i++) {
        double magnitude = sqrt(pow(data[i].x, 2) + 
                              pow(data[i].y, 2) + 
                              pow(data[i].z, 2));
        sum += magnitude;
        sum_sq += pow(magnitude, 2);
    }
    
    double mean = sum / count;
    return (sum_sq / count) - pow(mean, 2);
}

const char* detect_motion(MotionDetector *detector) {
    if (!detector->buffer_full && detector->buffer_index < WINDOW_SIZE - 1) {
        return "Insufficient data";
    }
    
    int count = detector->buffer_full ? WINDOW_SIZE : detector->buffer_index + 1;
    double variance = calculate_variance(detector->buffer, count);
    
    return variance > MOVEMENT_THRESHOLD ? "Moving" : "Stationary";
}

void add_accel_data(MotionDetector *detector, AccelData data) {
    detector->buffer[detector->buffer_index] = data;
    detector->buffer_index = (detector->buffer_index + 1) % WINDOW_SIZE;
    if (!detector->buffer_full && detector->buffer_index == 0) {
        detector->buffer_full = 1;
    }
}

int parse_accel_data(uint8_t *data, size_t size, AccelData *accel_data) {
    if (size < 36) {
        return 0;
    }
    
 
    uint16_t mfg_id = (data[26] << 8) | data[27];
    if (mfg_id != MANUFACTURER_ID) {
        return 0;
    }
    

    accel_data->x = (int8_t)data[28];
    accel_data->y = (int8_t)data[30];
    accel_data->z = (int8_t)data[32];
    
    return 1;
}


void print_timestamp() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm *tm = localtime(&ts.tv_sec);
    printf("[%02d:%02d:%02d.%03ld] ", 
           tm->tm_hour, tm->tm_min, tm->tm_sec, ts.tv_nsec / 1000000);
}

int main() {
    int device_id = hci_get_route(NULL);
    int device_handle = hci_open_dev(device_id);
    
    if (device_id < 0 || device_handle < 0) {
        perror("Opening socket failed");
        return 1;
    }
    
    
    MotionDetector detector;
    init_motion_detector(&detector);
    

    uint8_t scan_type = 0x00; 
    uint16_t interval = htobs(0x0010);
    uint16_t window = htobs(0x0010);
    uint8_t own_type = LE_PUBLIC_ADDRESS;
    uint8_t filter_policy = 0x00;
    
    if (hci_le_set_scan_parameters(device_handle, scan_type, interval, window,
                                 own_type, filter_policy, 1000) < 0) {
        perror("Set scan parameters failed");
        hci_close_dev(device_handle);
        return 1;
    }
    
 
    uint8_t filter_dup = 0x00;  
    if (hci_le_set_scan_enable(device_handle, 0x01, filter_dup, 1000) < 0) {
        perror("Enable scan failed");
        hci_close_dev(device_handle);
        return 1;
    }
    
    printf("Starting BLE scan for accelerometer data...\n");
    printf("Press Ctrl+C to stop\n\n");
    
    struct hci_filter old_filter, new_filter;
    socklen_t old_filter_len = sizeof(old_filter);
    getsockopt(device_handle, SOL_HCI, HCI_FILTER, &old_filter, &old_filter_len);
    
    hci_filter_clear(&new_filter);
    hci_filter_set_ptype(HCI_EVENT_PKT, &new_filter);
    hci_filter_set_event(EVT_LE_META_EVENT, &new_filter);
    setsockopt(device_handle, SOL_HCI, HCI_FILTER, &new_filter, sizeof(new_filter));
    
    unsigned char buf[HCI_MAX_EVENT_SIZE];
    while (1) {
        int len = read(device_handle, buf, sizeof(buf));
        if (len < 0) {
            perror("Read failed");
            break;
        }
        
        evt_le_meta_event *meta = (evt_le_meta_event *)(buf + HCI_EVENT_HDR_SIZE + 1);
        if (meta->subevent != EVT_LE_ADVERTISING_REPORT) {
            continue;
        }
        
        le_advertising_info *info = (le_advertising_info *)(meta->data + 1);
        
        AccelData accel_data;
        if (parse_accel_data(info->data, info->length, &accel_data)) {
            add_accel_data(&detector, accel_data);
            
            print_timestamp();
            char addr[18];
            ba2str(&info->bdaddr, addr);
            printf("Device: %s\n", addr);
            printf("Accelerometer Data (x,y,z): %d, %d, %d\n", 
                   accel_data.x, accel_data.y, accel_data.z);
            printf("Motion State: %s\n\n", detect_motion(&detector));
        }
    }
    
    setsockopt(device_handle, SOL_HCI, HCI_FILTER, &old_filter, sizeof(old_filter));
    hci_le_set_scan_enable(device_handle, 0x00, filter_dup, 1000);
    hci_close_dev(device_handle);
    
    return 0;
}