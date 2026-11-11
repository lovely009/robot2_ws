// Exercise the native renderer and UI against an offscreen context owned by the test.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <mujoco/mujoco.h>

static mjUI* make_ui(int rectid, int auxid) {
  mjUI* ui = calloc(1, sizeof(mjUI));
  if (ui) {
    ui->spacing = mjui_themeSpacing(0);
    ui->color = mjui_themeColor(0);
    ui->rectid = rectid;
    ui->auxid = auxid;
  }
  return ui;
}

int verify_native(const char* path, unsigned char* pixels, int width, int height, int scale) {
  char error[1024];
  mjModel* model = mj_loadXML(path, NULL, error, sizeof(error));
  if (!model) {
    fprintf(stderr, "%s\n", error);
    return 1;
  }
  model->vis.global.offwidth = width;
  model->vis.global.offheight = height;
  mjData* data = mj_makeData(model);
  mj_forward(model, data);
  mjrContext context;
  mjr_defaultContext(&context);
  mjr_makeContext(model, &context, scale);
  mjr_setBuffer(mjFB_OFFSCREEN, &context);
  mjUI* left = make_ui(1, 0);
  mjUI* right = make_ui(2, 1);
  if (!left || !right) {
    return 2;
  }
  int enabled = 1;
  int run = 0;
  mjtNum target = 0;
  mjuiDef left_def[] = {
    {mjITEM_SECTION, "File", mjSECT_OPEN, NULL, "AF"},
    {mjITEM_BUTTON, "Save xml", 1},
    {mjITEM_BUTTON, "Screenshot", 1},
    {mjITEM_BUTTON, "Quit", 1},
    {mjITEM_SECTION, "Option", mjSECT_CLOSED, NULL, "AO"},
    {mjITEM_SECTION, "Simulation", mjSECT_OPEN, NULL, "AS"},
    {mjITEM_RADIO, "", 1, &run, "Pause\nRun"},
    {mjITEM_BUTTON, "Reset", 1},
    {mjITEM_BUTTON, "Reload", 1},
    {mjITEM_BUTTON, "Align", 1},
    {mjITEM_SECTION, "Watch", mjSECT_CLOSED, NULL, "AW"},
    {mjITEM_SECTION, "Physics", mjSECT_CLOSED, NULL, "AP"},
    {mjITEM_SECTION, "Rendering", mjSECT_CLOSED, NULL, "AR"},
    {mjITEM_SECTION, "Visualization", mjSECT_CLOSED, NULL, "AV"},
    {mjITEM_SECTION, "Group enable", mjSECT_CLOSED, NULL, "AG"},
    {mjITEM_END}
  };
  mjuiDef right_def[] = {
    {mjITEM_SECTION, "Joint", mjSECT_CLOSED, NULL, "AJ"},
    {mjITEM_SECTION, "Control", mjSECT_OPEN, NULL, "AC"},
    {mjITEM_BUTTON, "Clear all", 1},
    {mjITEM_SLIDERNUM, "motor_01", 1, &target, "-1 1"},
    {mjITEM_SECTION, "Equality", mjSECT_OPEN, NULL, "AE"},
    {mjITEM_CHECKINT, "Contact", 1, &enabled},
    {mjITEM_END}
  };
  mjui_add(left, left_def);
  mjui_add(right, right_def);
  mjui_resize(left, &context);
  mjui_resize(right, &context);
  mjr_addAux(left->auxid, left->width, left->maxheight, left->spacing.samples, &context);
  mjr_addAux(right->auxid, right->width, right->maxheight, right->spacing.samples, &context);
  mjuiState state = {0};
  state.nrect = 3;
  state.rect[0] = (mjrRect){0, 0, width, height};
  state.rect[1] = (mjrRect){0, 0, left->width, height};
  state.rect[2] = (mjrRect){width - right->width, 0, right->width, height};
  mjrRect viewport = {left->width, 0, width - left->width - right->width, height};
  mjvScene scene;
  mjv_defaultScene(&scene);
  mjv_makeScene(model, &scene, 2000);
  mjvCamera camera;
  mjv_defaultFreeCamera(model, &camera);
  camera.distance *= 1.2;
  mjvOption option;
  mjv_defaultOption(&option);
  mjv_updateScene(model, data, &option, NULL, &camera, mjCAT_ALL, &scene);
  mjr_render(viewport, &scene, &context);
  mjui_update(-1, -1, left, &state, &context);
  mjui_update(-1, -1, right, &state, &context);
  mjui_render(left, &state, &context);
  mjui_render(right, &state, &context);
  mjr_readPixels(pixels, NULL, state.rect[0], &context);

  // Translate labels without changing the names used by native UI events.
  int result = strcmp(left->sect[0].name, "File") || strcmp(right->sect[1].name, "Control");
  state.type = mjEVENT_KEY;
  state.key = 'O';
  state.alt = 1;
  mjui_event(left, &state, &context);
  if (left->sect[1].state != mjSECT_OPEN || left->sect[0].state != mjSECT_CLOSED) {
    result = 3;
  }
  mjuiItem* slider = &right->sect[1].item[1];
  state.type = mjEVENT_PRESS;
  state.alt = 0;
  state.button = mjBUTTON_LEFT;
  state.left = 1;
  state.mouserect = 2;
  state.x = state.rect[2].left + slider->rect.left + .75 * slider->rect.width;
  state.y = height - right->height + slider->rect.bottom + .5 * slider->rect.height;
  mjuiItem* changed = mjui_event(right, &state, &context);
  if (changed != slider || target < .3 || target > .7) {
    result = 4;
  }
  mjv_freeScene(&scene);
  mjr_freeContext(&context);
  mj_deleteData(data);
  mj_deleteModel(model);
  free(left);
  free(right);
  return result;
}
