#include "test.h"
#include "fluidsynth.h"

int main(void)
{
#ifdef NETWORK_SUPPORT
    fluid_settings_t *settings1, *settings2;
    fluid_server_t *server1, *server2;
    int port1, port2;

    settings1 = new_fluid_settings();
    settings2 = new_fluid_settings();
    TEST_ASSERT(settings1 != NULL);
    TEST_ASSERT(settings2 != NULL);

    TEST_SUCCESS(fluid_settings_setint(settings1, "shell.port", 0));
    TEST_SUCCESS(fluid_settings_setint(settings2, "shell.port", 0));

    server1 = new_fluid_server2(settings1, NULL, NULL, NULL);
    TEST_ASSERT(server1 != NULL);
    TEST_SUCCESS(fluid_settings_getint(settings1, "shell.port", &port1));
    TEST_ASSERT(port1 >= 9800 && port1 <= 65535);

    server2 = new_fluid_server2(settings2, NULL, NULL, NULL);
    TEST_ASSERT(server2 != NULL);
    TEST_SUCCESS(fluid_settings_getint(settings2, "shell.port", &port2));
    TEST_ASSERT(port2 >= 9800 && port2 <= 65535);
    TEST_ASSERT(port1 != port2);

    (void)server1;
    (void)server2;
    (void)settings1;
    (void)settings2;
#endif

    return EXIT_SUCCESS;
}
