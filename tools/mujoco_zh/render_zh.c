// Native MuJoCo labels are translated when drawn; the UI stores its original labels.
#include "render_zh.h"

#include <ft2build.h>
#include FT_FREETYPE_H
#include <stdlib.h>
#include <string.h>
#include "render/classic/glad/glad.h"

typedef struct {
  const char* english;
  const char* chinese;
} Translation;

#include "translations.h"

typedef struct Glyph {
  unsigned int codepoint;
  int height;
  int width;
  int rows;
  int left;
  int top;
  int advance;
  unsigned char* pixels;
  struct Glyph* next;
} Glyph;

// Each viewer render thread owns its FreeType face and immutable glyph cache.
static _Thread_local FT_Library library;
static _Thread_local FT_Face face;
static _Thread_local Glyph* glyphs;
static _Thread_local int initialized;

static int initialize(void) {
  if (!initialized) {
    initialized = 1;
    const char* language = getenv("MUJOCO_LANGUAGE");
    const char* font = getenv("MUJOCO_ZH_FONT");
    if (language && !strcmp(language, "zh_CN") && font &&
        !FT_Init_FreeType(&library)) {
      if (FT_New_Face(library, font, 0, &face)) {
        FT_Done_FreeType(library);
        library = NULL;
        face = NULL;
      } else {
        for (int index = 1; index < face->num_faces; index++) {
          FT_Face candidate = NULL;
          if (!FT_New_Face(library, font, index, &candidate)) {
            if (candidate->family_name && strstr(candidate->family_name, "CJK SC")) {
              FT_Done_Face(face);
              face = candidate;
              break;
            }
            FT_Done_Face(candidate);
          }
        }
      }
    }
  }
  return face != NULL;
}

static const char* translate(const char* text) {
  if (!initialize()) {
    return NULL;
  }
  for (size_t i = 0; i < sizeof(translations) / sizeof(translations[0]); i++) {
    if (!strcmp(text, translations[i].english)) {
      return translations[i].chinese;
    }
  }
  for (const unsigned char* p = (const unsigned char*)text; *p; p++) {
    if (*p >= 128) {
      return text;
    }
  }
  return NULL;
}

static unsigned int next_codepoint(const unsigned char** position) {
  const unsigned char* p = *position;
  unsigned int value = *p++;
  int continuation = 0;
  if (value >= 0xC2 && value <= 0xDF) {
    value &= 0x1F;
    continuation = 1;
  } else if (value >= 0xE0 && value <= 0xEF) {
    value &= 0x0F;
    continuation = 2;
  } else if (value >= 0xF0 && value <= 0xF4) {
    value &= 0x07;
    continuation = 3;
  } else if (value >= 128) {
    value = 0xFFFD;
  }
  for (int i = 0; i < continuation; i++) {
    if ((*p & 0xC0) != 0x80) {
      *position = p;
      return 0xFFFD;
    }
    value = (value << 6) | (*p++ & 0x3F);
  }
  *position = p;
  return value;
}

static Glyph* get_glyph(unsigned int codepoint, int height) {
  for (Glyph* item = glyphs; item; item = item->next) {
    if (item->codepoint == codepoint && item->height == height) {
      return item;
    }
  }
  if (height < 4 || FT_Set_Pixel_Sizes(face, 0, height > 6 ? height - 2 : height) ||
      FT_Load_Char(face, codepoint, FT_LOAD_RENDER | FT_LOAD_TARGET_MONO | FT_LOAD_MONOCHROME)) {
    return NULL;
  }
  Glyph* item = calloc(1, sizeof(Glyph));
  if (!item) {
    return NULL;
  }
  item->codepoint = codepoint;
  item->height = height;
  item->width = face->glyph->bitmap.width;
  item->rows = face->glyph->bitmap.rows;
  item->left = face->glyph->bitmap_left;
  item->top = face->glyph->bitmap_top;
  item->advance = (face->glyph->advance.x + 32) >> 6;
  int stride = (item->width + 7) / 8;
  if (stride && item->rows) {
    item->pixels = malloc((size_t)stride * item->rows);
    if (!item->pixels) {
      free(item);
      return NULL;
    }
    for (int row = 0; row < item->rows; row++) {
      const unsigned char* source = face->glyph->bitmap.buffer +
          (item->rows - 1 - row) * face->glyph->bitmap.pitch;
      memcpy(item->pixels + row * stride, source, stride);
    }
  }
  item->next = glyphs;
  glyphs = item;
  return item;
}

int mjzh_textwidth(const char* text, int height) {
  const char* translated = translate(text);
  if (!translated) {
    return -1;
  }
  int width = 0;
  const unsigned char* p = (const unsigned char*)translated;
  while (*p) {
    Glyph* item = get_glyph(next_codepoint(&p), height);
    if (!item) {
      return -1;
    }
    width += item->advance;
  }
  return width;
}

int mjzh_draw(const char* text, int maxwidth, int height) {
  const char* translated = translate(text);
  if (!translated || mjzh_textwidth(text, height) < 0) {
    return 0;
  }
  glPushClientAttrib(GL_CLIENT_PIXEL_STORE_BIT);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
  glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
  glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
  glPixelStorei(GL_UNPACK_LSB_FIRST, GL_FALSE);
  int width = 0;
  const unsigned char* p = (const unsigned char*)translated;
  while (*p) {
    Glyph* item = get_glyph(next_codepoint(&p), height);
    if (item->advance > maxwidth - width) {
      break;
    }
    int vertical = height / 6 + item->top - item->rows;
    glBitmap(0, 0, 0, 0, item->left, vertical, NULL);
    if (item->pixels) {
      glBitmap(item->width, item->rows, 0, 0, 0, 0, item->pixels);
    }
    glBitmap(0, 0, 0, 0, item->advance - item->left, -vertical, NULL);
    width += item->advance;
  }
  glPopClientAttrib();
  return 1;
}
