// 4.22.1 0x36282cdd
// Generated by imageconverter. Please, do not edit!

#include <images/BitmapDatabase.hpp>
#include <touchgfx/Bitmap.hpp>

extern const unsigned char image_alternate_theme_images_widgets_button_regular_height_36_large_round_action[]; // BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_BUTTON_REGULAR_HEIGHT_36_LARGE_ROUND_ACTION_ID = 0, Size: 290x36 pixels
extern const unsigned char image_alternate_theme_images_widgets_button_regular_height_36_large_round_disabled[]; // BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_BUTTON_REGULAR_HEIGHT_36_LARGE_ROUND_DISABLED_ID = 1, Size: 290x36 pixels
extern const unsigned char image_alternate_theme_images_widgets_button_regular_height_36_large_round_pressed[]; // BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_BUTTON_REGULAR_HEIGHT_36_LARGE_ROUND_PRESSED_ID = 2, Size: 290x36 pixels
extern const unsigned char image_gottwald[]; // BITMAP_GOTTWALD_ID = 3, Size: 433x116 pixels
extern const unsigned char image_grauerhintergrund[]; // BITMAP_GRAUERHINTERGRUND_ID = 4, Size: 646x365 pixels
extern const unsigned char image_logo_lem[]; // BITMAP_LOGO_LEM_ID = 5, Size: 2560x767 pixels
extern const unsigned char image_racingteam_htl_logo[]; // BITMAP_RACINGTEAM_HTL_LOGO_ID = 6, Size: 1132x262 pixels
extern const unsigned char image_schrack_logo[]; // BITMAP_SCHRACK_LOGO_ID = 7, Size: 1024x354 pixels
extern const unsigned char image_traco_power[]; // BITMAP_TRACO_POWER_ID = 8, Size: 200x100 pixels

const touchgfx::Bitmap::BitmapData bitmap_database[] = {
    { image_alternate_theme_images_widgets_button_regular_height_36_large_round_action, 0, 290, 36, 17, 0, 256, ((uint8_t)touchgfx::Bitmap::ARGB8888) >> 3, 36, ((uint8_t)touchgfx::Bitmap::ARGB8888) & 0x7 },
    { image_alternate_theme_images_widgets_button_regular_height_36_large_round_disabled, 0, 290, 36, 17, 0, 256, ((uint8_t)touchgfx::Bitmap::ARGB8888) >> 3, 36, ((uint8_t)touchgfx::Bitmap::ARGB8888) & 0x7 },
    { image_alternate_theme_images_widgets_button_regular_height_36_large_round_pressed, 0, 290, 36, 17, 0, 256, ((uint8_t)touchgfx::Bitmap::ARGB8888) >> 3, 36, ((uint8_t)touchgfx::Bitmap::ARGB8888) & 0x7 },
    { image_gottwald, 0, 433, 116, 0, 0, 433, ((uint8_t)touchgfx::Bitmap::RGB565) >> 3, 116, ((uint8_t)touchgfx::Bitmap::RGB565) & 0x7 },
    { image_grauerhintergrund, 0, 646, 365, 0, 0, 646, ((uint8_t)touchgfx::Bitmap::RGB565) >> 3, 365, ((uint8_t)touchgfx::Bitmap::RGB565) & 0x7 },
    { image_logo_lem, 0, 2560, 767, 1345, 136, 76, ((uint8_t)touchgfx::Bitmap::ARGB8888) >> 3, 475, ((uint8_t)touchgfx::Bitmap::ARGB8888) & 0x7 },
    { image_racingteam_htl_logo, 0, 1132, 262, 0, 0, 1132, ((uint8_t)touchgfx::Bitmap::RGB565) >> 3, 262, ((uint8_t)touchgfx::Bitmap::RGB565) & 0x7 },
    { image_schrack_logo, 0, 1024, 354, 11, 194, 159, ((uint8_t)touchgfx::Bitmap::ARGB8888) >> 3, 20, ((uint8_t)touchgfx::Bitmap::ARGB8888) & 0x7 },
    { image_traco_power, 0, 200, 100, 18, 26, 18, ((uint8_t)touchgfx::Bitmap::ARGB8888) >> 3, 19, ((uint8_t)touchgfx::Bitmap::ARGB8888) & 0x7 }
};

namespace BitmapDatabase
{
const touchgfx::Bitmap::BitmapData* getInstance()
{
    return bitmap_database;
}

uint16_t getInstanceSize()
{
    return (uint16_t)(sizeof(bitmap_database) / sizeof(touchgfx::Bitmap::BitmapData));
}
} // namespace BitmapDatabase
