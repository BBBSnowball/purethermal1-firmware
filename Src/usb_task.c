
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "usb_device.h"


#include "pt.h"
#include "lepton.h"
#include "lepton_i2c.h"
#include "tmp007_i2c.h"
#include "mlx90614_i2c.h"
#include "usbd_uvc.h"
#include "usbd_uvc_if.h"


#include "tasks.h"

#include "project_config.h"

extern volatile uint8_t g_lepton_type_3;
extern volatile uint8_t g_uvc_stream_status;
extern volatile uint8_t g_telemetry_num_lines;
extern struct uvc_streaming_control videoCommitControl;

#ifdef OVERLAY_DEFAULT_ON
static int overlay_mode = 1;
#else
static int overlay_mode = 0;
#endif

void change_overlay_mode(void)
{
	overlay_mode = (overlay_mode+1) % 2;
}

static int last_frame_count;
static lepton_buffer *last_buffer;

#if defined(USART_DEBUG) || defined(GDB_SEMIHOSTING)
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif

void HAL_RCC_CSSCallback(void) {
	DEBUG_PRINTF("Oh no! HAL_RCC_CSSCallback()\r\n");
}

#include "ugui.h"

#if (defined(TMP007) && defined(TMP007_OVERLAY)) || (defined(MLX90614) && defined(MLX90614_OVERLAY)) || defined(SPLASHSCREEN_OVERLAY)
UG_GUI gui; 
static void pixel_set(UG_S16 x , UG_S16 y ,UG_COLOR c )
{
	y *= 2;
	if (y > IMAGE_NUM_LINES) return;
	if (x > FRAME_LINE_LENGTH) return;

	last_buffer->lines.rgb[y].data.image_data[x].g = c;

	//last_buffer->lines.rgb[y].data.image_data[x].r = c&0xff;
	//last_buffer->lines.rgb[y].data.image_data[x].g = (c>>8)&0xff;
	//last_buffer->lines.rgb[y].data.image_data[x].b = (c>>16)&0xff;

	//last_buffer->lines.rgb[y].data.image_data[x].r = c ? x*16 : 0;
	//last_buffer->lines.rgb[y].data.image_data[x].g = c ? y*16 : 0;
	//last_buffer->lines.rgb[y].data.image_data[x].b = c ? 0xff : 0;
}
#endif

static void draw_splash(int min, int max)
{
	int x_loc = 0;
	int y_loc = 0;

	//UG_PutChar(' ',x_loc,y_loc,max,min);
	x_loc+=8;
	//UG_PutChar(' ',x_loc,y_loc,max,min);
	x_loc+=8;
	//UG_PutChar(' ',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('P',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('U',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('R',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('E',x_loc,y_loc,max,min);
	x_loc+=8;
	//UG_PutChar(' ',x_loc,y_loc,max,min);
	x_loc+=8;
	//UG_PutChar(' ',x_loc,y_loc,max,min);
	x_loc+=8;
	//UG_PutChar(' ',x_loc,y_loc,max,min);

	x_loc=0;
	y_loc += 8;
	UG_PutChar('T',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('h',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('e',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('r',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('m',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('a',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('l',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar(' ',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('1',x_loc,y_loc,max,min);
	x_loc+=8;
	//UG_PutChar('0',x_loc,y_loc,max,min);

	x_loc=0;
	y_loc += 8;
	y_loc += 8;
	UG_PutChar('G',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('r',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('o',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('u',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('p',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('G',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('e',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('t',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('s',x_loc,y_loc,max,min);
	x_loc+=8;
	//UG_PutChar('0',x_loc,y_loc,max,min);

	x_loc=0;
	y_loc += 8;
	//UG_PutChar('G',x_loc,y_loc,max,min);
	x_loc+=8;
	//UG_PutChar('r',x_loc,y_loc,max,min);
	x_loc+=8;
	//UG_PutChar('o',x_loc,y_loc,max,min);
	x_loc+=8;
	//UG_PutChar('u',x_loc,y_loc,max,min);
	x_loc+=8;
	//UG_PutChar('p',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('.',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('c',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('o',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('m',x_loc,y_loc,max,min);
	x_loc+=8;
	//UG_PutChar('0',x_loc,y_loc,max,min);

}

volatile uint32_t debug_value = 42;

PT_THREAD( usb_task(struct pt *pt))
{
	static int temperature, temperature_milli;
	static uint16_t count = 0, i;

	static uint8_t uvc_header[2] = { 2, 0 };
	static uint32_t uvc_xmit_row = 0, uvc_xmit_plane = 0, uvc_xmit_seg = 0;
	static uint8_t packet[VIDEO_PACKET_SIZE_MAX];
	static int image_num_segments;

	(void)uvc_xmit_plane;  // avoid warning

	PT_BEGIN(pt);

#if (defined(TMP007) && defined(TMP007_OVERLAY)) || (defined(MLX90614) && defined(MLX90614_OVERLAY)) || defined(SPLASHSCREEN_OVERLAY)
	UG_Init(&gui,pixel_set,80,60);
	UG_FontSelect(&FONT_8X8);     
#endif

	image_num_segments = ((g_lepton_type_3 == 1) ? 4 : 1);

	while (1)
	{
		PT_WAIT_UNTIL(pt, (last_buffer = dequeue_lepton_buffer()) != NULL);

		uvc_xmit_row = 0;
		uvc_xmit_plane = 0;

		if (image_num_segments > 1)
		{
			if (uvc_xmit_seg == 0 && last_buffer->segment != 1)
			{
				// Skip this segment until we have the beginning of a frame
				continue;
			}
		}

		last_frame_count++;

#if (defined(MLX90614) && defined(MLX90614_OVERLAY))
		want_mlx90614_value = (overlay_mode == 1) && (videoCommitControl.bFormatIndex != VS_FMT_INDEX(Y16));
#endif

		if (videoCommitControl.bFormatIndex == VS_FMT_INDEX(Y16))
		{
			// never show splash for raw image data
		}
		else if (uvc_xmit_seg > 0)
		{
			// don't show any splash here - only do that in the first segment
		}
#ifdef SPLASHSCREEN_OVERLAY
		else if (((last_frame_count % 1800) > 0)   && ((last_frame_count % 1800) < 150)  )
		{
			if(overlay_mode == 1)
			{
				draw_splash(255, 0);
			}
		}
#endif
		else
		{

#if (defined(TMP007) && defined(TMP007_OVERLAY)) || (defined(MLX90614) && defined(MLX90614_OVERLAY))
			const int temperature_y_offset = 0;
			if(overlay_mode == 1 && has_last_milli_celsius())
			{
				temperature_milli = get_last_milli_celsius();
				#ifdef OVERLAY_FAHRENHEIT
					// "+5" is so we round instead of truncate
					temperature_milli = (temperature_milli * 18 + 5) / 10 + 32000;
				#endif
				temperature = temperature_milli/1000;

				if(temperature < 0)
				{
					UG_PutChar('-',0,temperature_y_offset,255,0);
					temperature = -temperature;
				}
				else if(((temperature/100)%10) != 0 ) 
				{
					UG_PutChar((temperature/100)%10 + '0',0,temperature_y_offset,255,0);
				}

				UG_PutChar((temperature/10)%10 + '0',8,temperature_y_offset,255,0);
				UG_PutChar(temperature%10 + '0',16,temperature_y_offset,255,0);
				UG_PutChar('.',24,temperature_y_offset,255,0);
				UG_PutChar(temperature_milli/100%10 + '0',32,temperature_y_offset,255,0);
				UG_PutChar(248,40,temperature_y_offset,255,0);
				#ifdef OVERLAY_FAHRENHEIT
					UG_PutChar('F',48,temperature_y_offset,255,0);
				#else
					UG_PutChar('C',48,temperature_y_offset,255,0);
				#endif
			}
			else if(overlay_mode == 1)
			{
				UG_PutChar('/',0,temperature_y_offset,255,0);
			}

#elif defined(SPLASHSCREEN_OVERLAY_COUNTER) || defined(DEBUG_IN_SPLASHSCREEN_OVERLAY)
#	ifdef DEBUG_IN_SPLASHSCREEN_OVERLAY
			uint32_t tmp = debug_value;
#	else
			int tmp = last_frame_count;
#	endif
			for (int i=0;i<10;i++) {
				UG_PutChar('0' + (tmp%10),FRAME_LINE_LENGTH-8-8*i,0,255,0);
				tmp /= 10;
			}
#endif
		}

    // perform stream initialization
    if (g_uvc_stream_status == 1)
    {
      DEBUG_PRINTF("Starting UVC stream...\r\n");

      uvc_header[0] = 2;
      uvc_header[1] = 0;
      UVC_Transmit_FS(uvc_header, 2);

      g_uvc_stream_status = 2;
    }

    // put image on stream as long as stream is open
    while (g_uvc_stream_status == 2)
    {
      count = 0;

      packet[count++] = uvc_header[0];
      packet[count++] = uvc_header[1];

      switch (videoCommitControl.bFormatIndex)
      {
        case VS_FMT_INDEX(GREY):
        {
          // while (uvc_xmit_row < 60 && count < VALDB(videoCommitControl.dwMaxPayloadTransferSize))
          while (uvc_xmit_row < IMAGE_NUM_LINES && count < g_uvc_stream_packet_size)
          {
            for (i = 0; i < FRAME_LINE_LENGTH; i++)
            {
              uint8_t val;
              rgb2yuv(last_buffer->lines.rgb[IMAGE_OFFSET_LINES + uvc_xmit_row].data.image_data[i],
                      &val, NULL, NULL);

              // AGC is on, so just use lower 8 bits
              packet[count++] = (uint8_t)val;
            }

            uvc_xmit_row++;
          }

          break;
        }
        case VS_FMT_INDEX(Y16):
        {
          // while (uvc_xmit_row < 60 && count < VALDB(videoCommitControl.dwMaxPayloadTransferSize))
          while (uvc_xmit_row < (IMAGE_NUM_LINES + g_telemetry_num_lines) && count < g_uvc_stream_packet_size)
          {
            for (i = 0; i < FRAME_LINE_LENGTH; i++)
            {
              uint16_t val = last_buffer->lines.y16[IMAGE_OFFSET_LINES + uvc_xmit_row].data.image_data[i];
              packet[count++] = (uint8_t)((val >> 0) & 0xFF);
              packet[count++] = (uint8_t)((val >> 8) & 0xFF);
            }

            uvc_xmit_row++;
          }

          break;
        }
        default:
        case VS_FMT_INDEX(YUYV):
        {
          while (uvc_xmit_row < IMAGE_NUM_LINES && count < g_uvc_stream_packet_size)
          {
            for (i = 0; i < FRAME_LINE_LENGTH; i++)
            {
              uint8_t y, uv;

              if ((i % 2) == 0)
                rgb2yuv(last_buffer->lines.rgb[IMAGE_OFFSET_LINES + uvc_xmit_row].data.image_data[i],
                        &y, &uv, NULL);
              else
                rgb2yuv(last_buffer->lines.rgb[IMAGE_OFFSET_LINES + uvc_xmit_row].data.image_data[i],
                        &y, NULL, &uv);

              packet[count++] = uv;
              packet[count++] = y;
            }

            uvc_xmit_row++;
          }

          break;
        }
        case VS_FMT_INDEX(BGR3):
        {
          while (uvc_xmit_row < IMAGE_NUM_LINES && count < g_uvc_stream_packet_size)
          {
            for (i = 0; i < FRAME_LINE_LENGTH; i++)
            {
              rgb_t rgb = last_buffer->lines.rgb[IMAGE_OFFSET_LINES + uvc_xmit_row].data.image_data[i];
              packet[count++] = rgb.b;
              packet[count++] = rgb.g;
              packet[count++] = rgb.r;
            }
            uvc_xmit_row++;
          }

          break;
        }
        case VS_FMT_INDEX(RGB565):
        {
          while (uvc_xmit_row < IMAGE_NUM_LINES && count < g_uvc_stream_packet_size)
          {
            for (i = 0; i < FRAME_LINE_LENGTH; i++)
            {
              rgb_t rgb = last_buffer->lines.rgb[IMAGE_OFFSET_LINES + uvc_xmit_row].data.image_data[i];
              uint16_t val =
                ((rgb.r >> 3) << 11) |
                ((rgb.g >> 2) <<  5) |
                ((rgb.b >> 3) <<  0);

              packet[count++] = (val >> 0) & 0xff;
              packet[count++] = (val >> 8) & 0xff;
            }
            uvc_xmit_row++;
          }

          break;
        }
      }

      // Check if image is done
      if (uvc_xmit_row == (IMAGE_NUM_LINES + g_telemetry_num_lines))
      {
        if (++uvc_xmit_seg == image_num_segments)
        {
          packet[1] |= 0x2; // Flag end of frame
        }
      }

      // printf("UVC_Transmit_FS(): packet=%p, count=%d\r\n", packet, count);
      // fflush(stdout);

      static int retries;
      retries = 1000;
      while (UVC_Transmit_FS(packet, count) == USBD_BUSY && g_uvc_stream_status == 2)
      {
        if (--retries == 0) {
//          DEBUG_PRINTF("UVC_Transmit_FS() failed (no one is acking)\r\n");
          break;
        }
        PT_YIELD(pt);
      }

      if (packet[1] & 0x2)
      {
        uvc_header[1] ^= 1; // toggle bit 0 for next new frame
        uvc_xmit_seg = 0;
        // DEBUG_PRINTF("Frame complete\r\n");
        break;
      }
      else if (uvc_xmit_row == (IMAGE_NUM_LINES + g_telemetry_num_lines))
      {
        break;
      }
      PT_YIELD(pt);
    }

	}
	PT_END(pt);
}
