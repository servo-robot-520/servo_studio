#include <gtest/gtest.h>

#include "app/params_backup.hpp"

namespace app {
    namespace {

        TEST(ParamsBackup, RoundTrip) {
            ParamsBackup backup;
            backup.model = "SC225";
            backup.id = 6;
            backup.params["p_coefficient"] = 15;
            backup.params["baud_rate"] = 0;
            backup.params["id"] = 6;

            const auto data = serializeBackup(backup);
            ParamsBackup parsed;
            QString error;
            ASSERT_TRUE(parseBackup(data, &parsed, &error)) << error.toStdString();
            EXPECT_EQ(parsed.model, "SC225");
            EXPECT_EQ(parsed.id, 6);
            EXPECT_EQ(parsed.params, backup.params);
        }

        TEST(ParamsBackup, RejectsGarbage) {
            ParamsBackup parsed;
            QString error;
            EXPECT_FALSE(parseBackup("not json", &parsed, &error));
            EXPECT_FALSE(parseBackup("{\"format\":\"other\",\"params\":{}}", &parsed, &error));
            EXPECT_FALSE(parseBackup("{\"format\":\"servo_studio.params\"}", &parsed, &error));
            EXPECT_FALSE(parseBackup(
                "{\"format\":\"servo_studio.params\",\"params\":{\"p\":\"x\"}}", &parsed, &error));
        }

        TEST(ParamsBackup, RestoreSkipsIdAndBaud) {
            // ID 与波特率写错会失联，恢复必须跳过。
            ParamsBackup backup;
            backup.params["id"] = 6;
            backup.params["baud_rate"] = 0;
            backup.params["p_coefficient"] = 15;

            const auto restorable = paramsForRestore(backup);
            EXPECT_EQ(restorable.size(), 1);
            EXPECT_TRUE(restorable.contains("p_coefficient"));
        }

    } // namespace
} // namespace app
