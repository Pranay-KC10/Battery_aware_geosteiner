#include <stdio.h>
#include <string.h>
#include <ctype.h>

int main() {
    FILE* fp = fopen("fresh_run/fsts.txt", "r");
    char line[1024];
    int line_num = 0;

    while (fgets(line, sizeof(line), fp)) {
        line_num++;
        line[strcspn(line, "\n")] = 0;

        if (line_num >= 8 && line_num <= 12) {
            printf("Line %d: '%s' (len=%d)\n", line_num, line, (int)strlen(line));
            if (strcmp(line, "15") == 0) {
                printf("  -> Found FST count!\n");
            }
        }
    }

    fclose(fp);
    return 0;
}