#pragma once
/*
  ОПИСАНИЕ БИТМАП-МАССИВОВ:
  -массив bat - изображение батареи
  -массив charge - изображение батареи с вилкой (зарядка)
  -массив low - уведомление о разряде батареи
  -массивы fps1 и fps2 - для анимации при парении
  -массивы logo1-logo5 - для анимации включения
  -массивы birdUp, birdDown - изображение птички для игры
  
  ИНСТРУМЕНТЫ:
  -Программа для конвертации изображений в массив: https://javl.github.io/image2cpp/
  -Редактор изображений: GIMP, Photoshop или любой векторный редактор для SVG
*/

// Улучшенная иконка батареи (10x10 пикселей)
const unsigned char bat[] PROGMEM = {
  0x3F, 0xC0, 0x7F, 0xE0, 0x60, 0x60, 0x60, 0x60, 0x7F, 0xE0, 
  0x7F, 0xE0, 0x7F, 0xE0, 0x7F, 0xE0, 0x7F, 0xE0, 0x3F, 0xC0
};

// Улучшенная иконка зарядки (10x10 пикселей)
const unsigned char charge[] PROGMEM = {
  0x3F, 0xC0, 0x7F, 0xE0, 0x6F, 0x60, 0x67, 0xE0, 0x73, 0xE0, 
  0x71, 0xE0, 0x70, 0x60, 0x7F, 0xE0, 0x7F, 0xE0, 0x3F, 0xC0
};

// 'low', 32x128px - уведомление о разряде батареи
const unsigned char low[] PROGMEM = {
  0x00, 0x00, 0x05, 0x02, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x02, 0x04, 0x00, 0x00, 0x02, 0x81, 
  0x00, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x84, 0x00, 0x00, 0x01, 0x85, 0x00, 0x00, 0x01, 0x42, 
  0x80, 0x00, 0x01, 0xc9, 0xe0, 0x00, 0x01, 0x49, 0xf0, 0x00, 0x01, 0xc1, 0x78, 0x00, 0x09, 0x4c, 
  0x3e, 0x00, 0x49, 0xd1, 0x1d, 0x00, 0x01, 0xd2, 0x89, 0x50, 0x05, 0x52, 0xac, 0xa0, 0xa9, 0xcc, 
  0xc7, 0x12, 0x01, 0xd0, 0xbd, 0xac, 0x11, 0xed, 0xac, 0xcb, 0x81, 0xd4, 0x9b, 0x76, 0xa3, 0xba, 
  0x9e, 0xbd, 0xfb, 0x70, 0x93, 0xfe, 0xd7, 0xd0, 0xdf, 0xf6, 0x3d, 0x77, 0xde, 0x3f, 0xcf, 0xc0, 
  0xf1, 0xc7, 0xb6, 0x80, 0x0f, 0x6b, 0x5d, 0x01, 0xfa, 0x95, 0xfe, 0x00, 0x05, 0xff, 0xbc, 0xc1, 
  0x38, 0x00, 0x06, 0x00, 0x82, 0x0a, 0xa1, 0x2b, 0x25, 0xff, 0xe0, 0x0a, 0x50, 0x20, 0x14, 0x00, 
  0x00, 0x02, 0xdb, 0x3a, 0x3f, 0xbf, 0x80, 0x00, 0x00, 0x00, 0x3c, 0x5f, 0x2a, 0x7c, 0x61, 0x80, 
  0x00, 0x82, 0x85, 0x61, 0x01, 0x00, 0x28, 0x1e, 0x0a, 0xbf, 0xfa, 0x21, 0x77, 0xe1, 0x83, 0xca, 
  0x04, 0x0f, 0x80, 0x30, 0xdf, 0xdf, 0xfc, 0x0f, 0x10, 0x3f, 0xf0, 0x80, 0x0a, 0xbf, 0xf7, 0x21, 
  0x77, 0xbf, 0xfe, 0x00, 0x20, 0x7f, 0xff, 0x01, 0x20, 0x7f, 0xfe, 0x00, 0x2e, 0x7f, 0xfa, 0x00, 
  0x21, 0xff, 0xfe, 0x00, 0xfc, 0x5f, 0xe0, 0x00, 0x0b, 0xe0, 0x00, 0x00, 0x20, 0x70, 0x00, 0x00, 
  0x20, 0x2c, 0x00, 0x00, 0x0e, 0xaa, 0x00, 0x00, 0x18, 0x0c, 0x00, 0x03, 0x05, 0xd0, 0x00, 0x04, 
  0x00, 0x07, 0x00, 0x07, 0xdc, 0x00, 0x00, 0x04, 0x17, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x0f, 0xac, 0x00, 0x00, 0x48, 0x00, 0x00, 0x00, 0x00, 0x9f, 0x01, 0x00, 0x0f, 0xd4, 0x7b, 0x80, 
  0x00, 0x03, 0xff, 0x80, 0x82, 0xff, 0xff, 0x80, 0x0e, 0x0f, 0xff, 0x03, 0x00, 0x0f, 0xf8, 0x04, 
  0x45, 0x4b, 0x80, 0x7f, 0x11, 0x2c, 0x60, 0x00, 0x42, 0x1f, 0xfe, 0x98, 0x15, 0x65, 0x41, 0x80, 
  0x00, 0x9a, 0xbc, 0x60, 0x02, 0x60, 0x00, 0x5f, 0x00, 0x8d, 0xd6, 0x00, 0xa0, 0x00, 0x00, 0x1f, 
  0x0d, 0xf7, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x01, 0x02, 0x08, 0x00, 0x28, 0x00, 0x02, 0xa0, 0x06, 
  0x81, 0x51, 0x5c, 0x90, 0x60, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x62, 0x0c, 0x2a, 0xda, 0x08, 
  0x01, 0xf0, 0x00, 0x04, 0x00, 0x07, 0xff, 0xf0, 0xc0, 0x08, 0x00, 0x48, 0x31, 0x00, 0x00, 0x0b, 
  0x04, 0x37, 0x00, 0x06, 0x0a, 0x80, 0x6f, 0x05, 0x31, 0xa0, 0x00, 0x03, 0x04, 0x30, 0x00, 0x01, 
  0x80, 0x4f, 0x80, 0x40, 0xc0, 0x00, 0x7e, 0x20, 0x30, 0x00, 0x00, 0x1c, 0x0c, 0x00, 0x00, 0x20, 
  0x03, 0x80, 0x00, 0x40, 0x00, 0x70, 0x00, 0x04, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x50, 
  0x00, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0x00, 0x54, 
  0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 
  0x0f, 0xc0, 0x00, 0x40, 0x00, 0x00, 0x00, 0x7c, 0x1f, 0xe0, 0x00, 0x40, 0x20, 0x10, 0x78, 0x40, 
  0x20, 0x10, 0x04, 0x40, 0x20, 0x10, 0x38, 0x40, 0x20, 0x10, 0x40, 0x7c, 0x20, 0x10, 0x38, 0x40, 
  0x20, 0x10, 0x04, 0x40, 0x2f, 0x50, 0x78, 0x04, 0x2f, 0x50, 0x00, 0x38, 0x20, 0x10, 0x38, 0x50, 
  0x20, 0x10, 0x44, 0x38, 0x20, 0x10, 0x44, 0x04, 0x20, 0x10, 0x38, 0x00, 0x20, 0x10, 0x04, 0x7c, 
  0x30, 0x30, 0x04, 0x54, 0x1f, 0xe0, 0x7c, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// 'fps1', 32x128px - первый кадр анимации при парении
const unsigned char fps1[] PROGMEM = {
  0x40, 0x40, 0x01, 0x03, 0x48, 0x40, 0x01, 0x13, 0x48, 0x40, 0x01, 0x13, 0x48, 0x00, 0x01, 0x10, 
  0x48, 0x00, 0x01, 0x10, 0x08, 0x00, 0x01, 0x10, 0x08, 0x00, 0x61, 0x10, 0x40, 0x7e, 0xe1, 0x10, 
  0x40, 0xff, 0xc1, 0x10, 0x41, 0x83, 0x81, 0x12, 0x43, 0x07, 0xc1, 0x12, 0x42, 0x0e, 0xc1, 0x12, 
  0x45, 0x9c, 0xc1, 0x12, 0x41, 0xf8, 0xc1, 0x12, 0x41, 0xf0, 0xc1, 0x12, 0x41, 0xf0, 0xc1, 0x12, 
  0x40, 0xf9, 0x81, 0x12, 0x40, 0x73, 0x80, 0x10, 0x40, 0x23, 0x80, 0x10, 0x40, 0x01, 0xe0, 0x00, 
  0x00, 0x00, 0x60, 0x01, 0x08, 0x00, 0x00, 0x48, 0x00, 0x00, 0x00, 0x48, 0x10, 0x00, 0x01, 0x00, 
  0x10, 0x00, 0x04, 0x02, 0x10, 0x80, 0x00, 0x02, 0x10, 0x80, 0x04, 0x02, 0x50, 0x80, 0x04, 0x32, 
  0x50, 0xa0, 0x04, 0x12, 0x50, 0x80, 0x04, 0x12, 0x50, 0x00, 0x00, 0x12, 0x50, 0x00, 0x00, 0x12, 
  0x50, 0x00, 0x00, 0x12, 0x50, 0x00, 0x00, 0x12, 0x50, 0x88, 0x48, 0x12, 0x10, 0x88, 0x48, 0x12, 
  0x10, 0x88, 0x48, 0x10, 0x10, 0x88, 0x08, 0x10, 0x00, 0x88, 0x08, 0x10, 0x00, 0x88, 0x08, 0x10, 
  0x00, 0x88, 0x08, 0x10, 0x00, 0x88, 0x08, 0x90, 0x04, 0x08, 0x48, 0x82, 0x44, 0x08, 0x08, 0x82, 
  0x44, 0x08, 0x08, 0x80, 0x04, 0x00, 0x08, 0x80, 0x04, 0x02, 0x08, 0x00, 0x04, 0x02, 0x08, 0x04, 
  0x44, 0x82, 0x00, 0x04, 0x44, 0x82, 0x00, 0x00, 0x44, 0x80, 0x40, 0x80, 0x44, 0x00, 0x40, 0x80, 
  0x40, 0x00, 0x40, 0x80, 0x40, 0x00, 0x40, 0x84, 0x48, 0x00, 0x44, 0x84, 0x08, 0x04, 0x04, 0x84, 
  0x08, 0x84, 0x04, 0x80, 0x08, 0x80, 0x04, 0x80, 0x08, 0x80, 0x04, 0x00, 0x08, 0x80, 0x40, 0x00, 
  0x48, 0x80, 0x40, 0x01, 0x48, 0x80, 0x00, 0x11, 0x48, 0x80, 0x00, 0x11, 0x48, 0x80, 0x00, 0x11, 
  0x48, 0x80, 0x00, 0x11, 0x40, 0x80, 0x00, 0x11, 0x42, 0x00, 0x00, 0x11, 0x42, 0x00, 0x00, 0x01, 
  0x52, 0x00, 0x00, 0x01, 0x52, 0x00, 0x00, 0x01, 0x52, 0x00, 0x60, 0x01, 0x12, 0x00, 0x60, 0x01, 
  0x12, 0x00, 0xb0, 0x00, 0x12, 0x00, 0xf0, 0x00, 0x10, 0x00, 0xf0, 0x00, 0x10, 0x01, 0xf8, 0x00, 
  0x10, 0x02, 0x78, 0x00, 0x10, 0x04, 0x38, 0x00, 0x10, 0x04, 0x3e, 0x00, 0x12, 0x04, 0x3f, 0x04, 
  0x12, 0x04, 0x3f, 0x04, 0x02, 0x04, 0x7f, 0x04, 0x02, 0x03, 0xff, 0x04, 0x42, 0x03, 0xbf, 0x05, 
  0x42, 0x06, 0xff, 0x05, 0x42, 0x0f, 0xfd, 0x05, 0x42, 0x0f, 0xfd, 0x01, 0x02, 0x0f, 0xed, 0x01, 
  0x02, 0x0f, 0xfd, 0x01, 0x12, 0x0f, 0xef, 0x01, 0x12, 0x0b, 0xec, 0x01, 0x10, 0x0b, 0xfc, 0x00, 
  0x10, 0x0b, 0xf8, 0x00, 0x10, 0x0b, 0xf0, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x08, 0xc0, 0x00, 
  0x00, 0x09, 0xe0, 0x00, 0x48, 0x01, 0xe0, 0x00, 0x40, 0x01, 0xe0, 0x10, 0x40, 0x01, 0xe0, 0x10, 
  0x41, 0x01, 0xc0, 0x10, 0x41, 0x01, 0xc0, 0x10, 0x49, 0x01, 0xc0, 0x10, 0x49, 0x00, 0xc0, 0x10, 
  0x09, 0x00, 0xc0, 0x90, 0x08, 0x00, 0xc0, 0x90, 0x08, 0x00, 0xc0, 0x92, 0x08, 0x00, 0x80, 0x92, 
  0x00, 0x00, 0x80, 0x02, 0x00, 0x00, 0x80, 0x02, 0x20, 0x44, 0x00, 0x00, 0x20, 0x44, 0x02, 0x00, 
  0x20, 0x44, 0x02, 0x08, 0x20, 0x44, 0x22, 0x08, 0x20, 0x44, 0x02, 0x08, 0x24, 0x44, 0x02, 0x48, 
  0x24, 0x44, 0x02, 0x48, 0x24, 0x44, 0x02, 0x48, 0x24, 0x40, 0x20, 0x48, 0x24, 0x40, 0x20, 0x48, 
  0x24, 0x00, 0x20, 0x40, 0x24, 0x00, 0x20, 0x40, 0x20, 0x42, 0x20, 0x40, 0x00, 0x42, 0x02, 0x40, 
  0x01, 0x42, 0x02, 0x02, 0x31, 0x00, 0x02, 0x02, 0x23, 0x00, 0x00, 0x02, 0x20, 0x00, 0x00, 0x00
};

// 'fps2', 32x128px - второй кадр анимации при парении
const unsigned char fps2[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x20, 0x00, 0x00, 0x04, 0x20, 0x40, 0x00, 0x04, 0x00, 0x40, 
  0x00, 0x80, 0x00, 0x44, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x60, 0x00, 0x80, 0x7e, 0xe0, 0x00, 
  0x80, 0xff, 0xc0, 0x00, 0xa1, 0x83, 0x84, 0x80, 0x03, 0x07, 0xc4, 0x80, 0x02, 0x0e, 0xc4, 0x82, 
  0x25, 0x9c, 0xc4, 0x00, 0x21, 0xf8, 0xc4, 0x04, 0x21, 0xf0, 0xc0, 0x04, 0x11, 0xf0, 0xc0, 0x00, 
  0x00, 0xf9, 0x80, 0x80, 0x00, 0x73, 0x80, 0x80, 0x00, 0x23, 0x80, 0x80, 0x00, 0x01, 0xe0, 0x00, 
  0x10, 0x00, 0x60, 0x20, 0x00, 0x00, 0x00, 0x20, 0x02, 0x00, 0x00, 0x20, 0x00, 0x00, 0x08, 0x20, 
  0x02, 0x20, 0x08, 0x20, 0x02, 0x21, 0x08, 0x20, 0x22, 0x21, 0x08, 0x22, 0x22, 0x21, 0x00, 0x22, 
  0x22, 0x20, 0x01, 0x22, 0x22, 0x20, 0x01, 0x02, 0x02, 0x20, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 
  0x02, 0x01, 0x00, 0x40, 0x02, 0x01, 0x08, 0x40, 0x02, 0x01, 0x00, 0x40, 0x20, 0x01, 0x04, 0x44, 
  0x20, 0x01, 0x04, 0x44, 0x20, 0x20, 0x04, 0x40, 0x20, 0x20, 0x04, 0x00, 0x21, 0x00, 0x04, 0x01, 
  0x20, 0x00, 0x00, 0x09, 0x00, 0x00, 0x20, 0x09, 0x00, 0x04, 0x20, 0x09, 0x00, 0x44, 0x20, 0x08, 
  0x00, 0x44, 0x20, 0x08, 0x02, 0x44, 0x21, 0x08, 0x02, 0x44, 0x21, 0x00, 0x02, 0x00, 0x01, 0x00, 
  0x12, 0x00, 0x01, 0x20, 0x10, 0x00, 0x00, 0x22, 0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x40, 0x22, 
  0x00, 0x10, 0x44, 0x22, 0x00, 0x10, 0x44, 0x22, 0x02, 0x10, 0x44, 0x00, 0x02, 0x10, 0x00, 0x00, 
  0xc2, 0x10, 0x00, 0x00, 0xc0, 0x10, 0x00, 0x28, 0xc0, 0x10, 0x00, 0x28, 0x00, 0x10, 0x04, 0x28, 
  0x00, 0x10, 0x04, 0x28, 0x00, 0x10, 0x04, 0x28, 0x01, 0x00, 0x04, 0x28, 0x11, 0x00, 0x00, 0x20, 
  0x91, 0x00, 0x00, 0x20, 0x80, 0x00, 0x00, 0x20, 0x80, 0x00, 0x00, 0x04, 0x84, 0x00, 0x00, 0x04, 
  0x84, 0x00, 0x00, 0x04, 0x04, 0x00, 0x00, 0x04, 0x04, 0x00, 0x60, 0x00, 0x04, 0x00, 0x60, 0x00, 
  0x00, 0x00, 0xb0, 0x00, 0x10, 0x00, 0xf0, 0x00, 0x00, 0x00, 0xf0, 0x01, 0x00, 0x01, 0xf8, 0x01, 
  0x00, 0x02, 0x78, 0x01, 0x44, 0x04, 0x38, 0x01, 0x44, 0x04, 0x3e, 0x01, 0x04, 0x04, 0x3f, 0x01, 
  0x04, 0x04, 0x3f, 0x01, 0x04, 0x04, 0x7f, 0x01, 0x04, 0x03, 0xff, 0x05, 0x04, 0x03, 0xbf, 0x05, 
  0x04, 0x06, 0xff, 0x05, 0x04, 0x0f, 0xfd, 0x05, 0x04, 0x0f, 0xfd, 0x05, 0x44, 0x0f, 0xed, 0x05, 
  0x44, 0x0f, 0xfd, 0x05, 0x44, 0x0f, 0xef, 0x05, 0x44, 0x0b, 0xec, 0x01, 0x40, 0x0b, 0xfc, 0x01, 
  0x40, 0x0b, 0xf8, 0x01, 0x40, 0x0b, 0xf0, 0x01, 0x40, 0x08, 0x00, 0x01, 0x40, 0x08, 0xc0, 0x00, 
  0x40, 0x09, 0xe0, 0x10, 0x50, 0x01, 0xe0, 0x10, 0x50, 0x01, 0xe0, 0x10, 0x50, 0x01, 0xe0, 0x10, 
  0x50, 0x00, 0xe0, 0x11, 0x50, 0x00, 0xe0, 0x14, 0x50, 0x00, 0xe0, 0x14, 0x42, 0x00, 0xc0, 0x14, 
  0x42, 0x00, 0xc0, 0x14, 0x42, 0x00, 0xc0, 0x14, 0x42, 0x00, 0xc0, 0x14, 0x42, 0x00, 0x40, 0x14, 
  0x42, 0x00, 0x40, 0x14, 0x42, 0x00, 0x40, 0x14, 0x42, 0x00, 0x00, 0x05, 0x42, 0x00, 0x00, 0x05, 
  0x42, 0x00, 0x00, 0x05, 0x42, 0x00, 0x00, 0x05, 0x42, 0x00, 0x00, 0x05, 0x42, 0x00, 0x00, 0x25, 
  0x42, 0x20, 0x00, 0x25, 0x42, 0x20, 0x00, 0x25, 0x02, 0x20, 0x00, 0x25, 0x12, 0x20, 0x00, 0x05, 
  0x12, 0x20, 0x00, 0x04, 0x12, 0x20, 0x00, 0x84, 0x10, 0x20, 0x00, 0x84, 0x10, 0x20, 0x00, 0x84, 
  0x90, 0x20, 0x00, 0x84, 0x80, 0x20, 0x00, 0x80, 0x80, 0x00, 0x00, 0x80, 0x80, 0x20, 0x00, 0x80
};

// Новые улучшенные логотипы для анимации включения
// '1', 32x32px - стартовый логотип
const unsigned char logo1[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFC, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x03, 0xC0, 0x03, 0xC0, 
  0x07, 0x00, 0x00, 0xE0, 0x0E, 0x00, 0x00, 0x70, 0x1C, 0x38, 0x1C, 0x38, 0x38, 0x7C, 0x3E, 0x1C, 
  0x30, 0xC6, 0x63, 0x0C, 0x71, 0x83, 0xC1, 0x8E, 0x61, 0x83, 0xC1, 0x86, 0x61, 0x83, 0xC1, 0x86, 
  0xE1, 0x83, 0xC1, 0x87, 0xC1, 0x83, 0xC1, 0x83, 0xC3, 0x00, 0x00, 0xC3, 0x83, 0x00, 0x00, 0xC1, 
  0x83, 0x1F, 0xF8, 0xC1, 0x83, 0x3F, 0xFC, 0xC1, 0x83, 0x7F, 0xFE, 0xC1, 0x83, 0xFF, 0xFF, 0xC1, 
  0x81, 0xFF, 0xFF, 0x81, 0x80, 0xFF, 0xFF, 0x01, 0xC0, 0x7F, 0xFE, 0x03, 0x60, 0x1F, 0xF8, 0x06, 
  0x30, 0x07, 0xE0, 0x0C, 0x18, 0x00, 0x00, 0x18, 0x0C, 0x00, 0x00, 0x30, 0x06, 0x00, 0x00, 0x60, 
  0x03, 0x80, 0x01, 0xC0, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x3F, 0xFC, 0x00
};

// '2', 32x32px - логотип, второй кадр
const unsigned char logo2[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFC, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x03, 0xC0, 0x03, 0xC0, 
  0x07, 0x00, 0x00, 0xE0, 0x0E, 0x00, 0x00, 0x70, 0x1C, 0x00, 0x00, 0x38, 0x38, 0x0F, 0xF0, 0x1C, 
  0x30, 0x3F, 0xFC, 0x0C, 0x71, 0x70, 0x0E, 0x8E, 0x61, 0xE0, 0x07, 0x86, 0x61, 0xC0, 0x03, 0x86, 
  0xE1, 0xC0, 0x03, 0x87, 0xC1, 0xC0, 0x03, 0x83, 0xC3, 0xE0, 0x07, 0xC3, 0x83, 0x70, 0x0E, 0xC1, 
  0x83, 0x3F, 0xFC, 0xC1, 0x83, 0x0F, 0xF0, 0xC1, 0x83, 0x00, 0x00, 0xC1, 0x83, 0x00, 0x00, 0xC1, 
  0x81, 0xC0, 0x03, 0x81, 0x80, 0xC0, 0x03, 0x01, 0xC0, 0xE0, 0x07, 0x03, 0x60, 0x70, 0x0E, 0x06, 
  0x30, 0x3F, 0xFC, 0x0C, 0x18, 0x0F, 0xF0, 0x18, 0x0C, 0x00, 0x00, 0x30, 0x06, 0x00, 0x00, 0x60, 
  0x03, 0x80, 0x01, 0xC0, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x3F, 0xFC, 0x00
};

// '3', 32x32px - логотип, третий кадр
const unsigned char logo3[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFC, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x03, 0xC0, 0x03, 0xC0, 
  0x07, 0x00, 0x00, 0xE0, 0x0E, 0x00, 0x00, 0x70, 0x1C, 0x00, 0x00, 0x38, 0x38, 0x0F, 0xF0, 0x1C, 
  0x30, 0x3F, 0xFC, 0x0C, 0x71, 0x70, 0x0E, 0x0E, 0x61, 0xE0, 0x07, 0x06, 0x61, 0xC0, 0x03, 0x86, 
  0x61, 0xC0, 0x03, 0x86, 0x61, 0xE0, 0x07, 0x86, 
  0x61, 0x70, 0x0E, 0x86, 
  0x61, 0x3F, 0xFC, 0x86, 0x61, 0x0F, 0xF0, 0x86, 0x61, 0x00, 0x00, 0x86, 0x61, 0xC0, 0x03, 0x86, 
  0x61, 0xC0, 0x03, 0x86, 0x61, 0xC0, 0x03, 0x86, 0x61, 0xE0, 0x07, 0x86, 0x30, 0x70, 0x0E, 0x0C, 
  0x38, 0x3F, 0xFC, 0x1C, 0x1C, 0x0F, 0xF0, 0x38, 0x0E, 0x00, 0x00, 0x70, 0x07, 0x00, 0x00, 0xE0, 
  0x03, 0x80, 0x01, 0xC0, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x3F, 0xFC, 0x00
};

// '4', 32x32px - логотип, четвертый кадр
const unsigned char logo4[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFC, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x03, 0xC0, 0x03, 0xC0, 
  0x07, 0x00, 0x00, 0xE0, 0x0E, 0x00, 0x00, 0x70, 0x1C, 0x00, 0x00, 0x38, 0x38, 0x00, 0x00, 0x1C, 
  0x30, 0x00, 0x00, 0x0C, 0x70, 0x03, 0xC0, 0x0E, 0x60, 0x0F, 0xF0, 0x06, 0x60, 0x1C, 0x38, 0x06, 
  0x60, 0x18, 0x18, 0x06, 0x60, 0x30, 0x0C, 0x06, 0x60, 0x30, 0x0C, 0x06, 0x60, 0x30, 0x0C, 0x06, 
  0x60, 0x30, 0x0C, 0x06, 0x60, 0x30, 0x0C, 0x06, 0x60, 0x30, 0x0C, 0x06, 0x60, 0x30, 0x0C, 0x06, 
  0x60, 0x30, 0x0C, 0x06, 0x60, 0x18, 0x18, 0x06, 0x60, 0x1C, 0x38, 0x06, 0x70, 0x0F, 0xF0, 0x0E, 
  0x30, 0x03, 0xC0, 0x0C, 0x18, 0x00, 0x00, 0x18, 0x0C, 0x00, 0x00, 0x30, 0x06, 0x00, 0x00, 0x60, 
  0x03, 0x80, 0x01, 0xC0, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x3F, 0xFC, 0x00
};

// '5', 32x32px - логотип, пятый кадр (финальный)
const unsigned char logo5[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFC, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x03, 0xC0, 0x03, 0xC0, 
  0x07, 0x00, 0x00, 0xE0, 0x0E, 0x00, 0x00, 0x70, 0x1C, 0x20, 0x04, 0x38, 0x38, 0x70, 0x0E, 0x1C, 
  0x30, 0xF8, 0x1F, 0x0C, 0x71, 0xBC, 0x3D, 0x8E, 0x63, 0x0E, 0x70, 0xC6, 0x66, 0x07, 0xE0, 0x66, 
  0x6C, 0x03, 0xC0, 0x36, 0x6C, 0x01, 0x80, 0x36, 0x6C, 0x01, 0x80, 0x36, 0x6C, 0x01, 0x80, 0x36, 
  0x6C, 0x01, 0x80, 0x36, 0x6C, 0x01, 0x80, 0x36, 0x6C, 0x01, 0x80, 0x36, 0x66, 0x01, 0x80, 0x66, 
  0x63, 0x01, 0x80, 0xC6, 0x61, 0x81, 0x81, 0x86, 0x60, 0xC1, 0x83, 0x06, 0x70, 0x61, 0x86, 0x0E, 
  0x30, 0x31, 0x8C, 0x0C, 0x18, 0x1B, 0xD8, 0x18, 0x0C, 0x0F, 0xF0, 0x30, 0x06, 0x00, 0x00, 0x60, 
  0x03, 0x80, 0x01, 0xC0, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x3F, 0xFC, 0x00
};

// Массив всех логотипов для удобства
const unsigned char* logoArr[5] = {
  logo1,
  logo2,
  logo3,
  logo4,
  logo5
};

// Улучшенная анимация птички для FlappyBird
// 'birdUp', 16x14px - птичка с крыльями вверх
const unsigned char birdUp[] PROGMEM = {
  0x03, 0xC0, 0x0F, 0xF0, 0x1F, 0xF8, 0x3F, 0xFC, 0x3F, 0xFC, 0x7F, 0xFE, 
  0x7F, 0xFE, 0x7F, 0xFE, 0x7F, 0xFE, 0x3F, 0xFC, 0x1F, 0xF8, 0x0F, 0xF0, 
  0x07, 0xE0, 0x01, 0x80
};

// 'birdDown', 14x15px - птичка с крыльями вниз
const unsigned char birdDown[] PROGMEM = {
  0x07, 0x80, 0x1F, 0xE0, 0x3F, 0xF0, 0x7F, 0xF8, 0x7F, 0xF8, 0x7F, 0xF8, 
  0x7F, 0xF8, 0x7F, 0xF8, 0x3F, 0xF0, 0x1F, 0xE0, 0x0F, 0xC0, 0x07, 0x80, 
  0x03, 0x00, 0x03, 0x00, 0x01, 0x00
};

// 'arrows', 28x9px - стрелки для меню
const unsigned char arrows[] PROGMEM = {
  0x0F, 0xFE, 0x00, 0x3F, 0xFF, 0x80, 0x7F, 0xFF, 0xC0, 0x7F, 0xFF, 0xC0, 
  0x3F, 0xFF, 0x80, 0x1F, 0xFF, 0x00, 0x0F, 0xFE, 0x00, 0x07, 0xFC, 0x00, 
  0x01, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x01, 0xF0, 0x00, 0x07, 0xFC, 0x00
};

// Массив изображений для анимации парения
const unsigned char* arr[2] = {
  fps1,
  fps2
}; 