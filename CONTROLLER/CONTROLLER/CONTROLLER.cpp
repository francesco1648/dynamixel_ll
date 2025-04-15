#include <stdio.h>
#include <windows.h>

#define BAUD_RATE CBR_115200

void sendSerial(HANDLE hSerial, char key) {
    DWORD bytesWritten;
    char buffer[2] = { key, '\n' };
    if (WriteFile(hSerial, buffer, 2, &bytesWritten, NULL)) {
        printf("Tasto '%c' (ASCII %d) inviato correttamente.\n", key, (int)key);
    }
    else {
        fprintf(stderr, "Errore nell'invio del tasto '%c'. Codice errore: %d\n", key, GetLastError());
    }
}

int main() {
    char comPort[10];
    printf("Inserisci la porta COM (es. COM9): ");
    scanf_s("%9s", comPort, (unsigned int)sizeof(comPort)); // Usa scanf_s per sicurezza

    char fullComPort[15];
    snprintf(fullComPort, sizeof(fullComPort), "\\\\.\\%s", comPort);

    HANDLE hSerial = CreateFileA(fullComPort, GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (hSerial == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Errore nell'apertura della porta seriale. Codice errore: %d\n", GetLastError());
        return 1;
    }

    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams)) {
        fprintf(stderr, "Errore nella lettura dello stato della porta seriale. Codice errore: %d\n", GetLastError());
        CloseHandle(hSerial);
        return 1;
    }
    dcbSerialParams.BaudRate = BAUD_RATE;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    if (!SetCommState(hSerial, &dcbSerialParams)) {
        fprintf(stderr, "Errore nella configurazione della porta seriale. Codice errore: %d\n", GetLastError());
        CloseHandle(hSerial);
        return 1;
    }

    printf("Monitoraggio tastiera attivo. Premi ESC per uscire.\n");

    while (1) {
        if (GetAsyncKeyState(VK_ESCAPE) & 0x8000) {
            printf("Tasto ESC premuto. Uscita dal programma.\n");
            break;
        }

        for (char key = 32; key <= 126; key++) {
            if (GetAsyncKeyState(key) & 0x8000) {
                sendSerial(hSerial, key);
                Sleep(30);
                printf("Il valore mandato è: %c (ASCII: %d)\n", key, (int)key);
            }
        }

        Sleep(50);
    }

    CloseHandle(hSerial);
    printf("Uscito dal programma.\n");
    return 0;
}
