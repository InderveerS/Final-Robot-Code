#pragma once

// The competition mission: a blocking move/turn/follow sequence. Runs once,
// then parks with the motors stopped. Start it as a task from setup().
void missionTask(void* pvParameters);
