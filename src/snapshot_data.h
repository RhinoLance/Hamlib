#ifndef _SNAPSHOT_DATA_H
#define _SNAPSHOT_DATA_H

void snapshot_init();
int snapshot_serialize(size_t buffer_length, char *buffer, RIG *rig, struct rig_spectrum_line *spectrum_line);

#endif
