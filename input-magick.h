/* input-magick.h: import files via image magick */

#ifndef INPUT_MAGICK_H
#define INPUT_MAGICK_H

#include "input.h"

at_bitmap_type magick_load_image (at_string filename,
				  at_msg_func msg_func, 
				  at_address msg_data);

#endif /* not INPUT_MAGICK_H */
