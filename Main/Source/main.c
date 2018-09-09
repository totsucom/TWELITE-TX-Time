/*
 * 送信にかかる時間を測定
 */

#include <AppHardwareApi.h>
#include "utils.h"
#include "ToCoNet.h"
#include "serial.h"
#include "string.h"
#include "sprintf.h"
#include "ToCoNet_mod_prototype.h"  // ToCoNet モジュール定義(無線で使う)

// 状態定義
typedef enum
{
    //組み込み済みの状態
    //E_STATE_IDLE        //初期状態
    //E_STATE_RUNNING     //実行中
    //E_STATE_FINISHED    //完了

    //ユーザー定義の状態
    E_STATE_APP_BASE = ToCoNet_STATE_APP_BASE, //enum開始番号の決定のために必要
    E_STATE_APP_WAIT_TX,            //送信完了待ち
    E_STATE_APP_WAIT_TIME           //送信開始待ち
} teStateApp;

// 無線通信用パラメータ(通信先と合わせておくこと)
#define APP_ID      0x67720103
#define CHANNEL     18

//送信中の電圧を測定
//#define MEASURE_VOLTAGE

//送信時間測定に関する変化させるパラメータ
//送信文字数
#define CHARS_MIN       0
#define CHARS_MAX       100
#define CHARS_STEP      20
//再送信回数（実際の送信回数はこの値+1となる）
#define RETRY_MIN       0
#define RETRY_MAX       8
#define RETRY_STEP      2
//再送信間隔[ms]
#define T_RETRY_MIN     1
#define T_RETRY_MAX     10
#define T_RETRY_STEP    2
//サンプリング回数（ばらつくので平均をとる）
#define N_SAMPLING      5

#define UART_BAUD 115200 	        // シリアルのボーレート
static tsFILE sSerStream;           // シリアル用ストリーム
static tsSerialPortSetup sSerPort;  // シリアルポートデスクリプタ

// シリアルにメッセージを出力する
#define debug(...) vfPrintf(&sSerStream, LB __VA_ARGS__)

// 送信開始時間を記憶
uint32 t0;

uint32 adcMin, adcMax;

// 無線で送信する
static bool_t sendBroadcast(uint8 nCHars, uint8 nRetry, uint16 tRetry)
{
    tsTxDataApp tsTx;
    memset(&tsTx, 0, sizeof(tsTxDataApp));

    tsTx.u32SrcAddr = ToCoNet_u32GetSerial();//チップのS/N
    tsTx.u32DstAddr = TOCONET_MAC_ADDR_BROADCAST;

    tsTx.bAckReq = FALSE;
    tsTx.u8Retry = nRetry; // 送信失敗時は 2回再送
    tsTx.u8CbId = 1;
    tsTx.u8Seq = 1;
    tsTx.u8Cmd = TOCONET_PACKET_CMD_APP_DATA;
    tsTx.u16RetryDur = tRetry;   // 再送間隔[ms]
    tsTx.u16DelayMax = 0;  // 送信開始タイミングにブレを作る(最大0ms)

    memset(tsTx.auData, 0x20, nCHars);
    tsTx.u8Len = nCHars;

    t0 = u32TickCount_ms;

    // 送信
    return ToCoNet_bMacTxReq(&tsTx);
}

// デバッグ出力用に UART を初期化
static void vSerialInit() {
    static uint8 au8SerialTxBuffer[96];
    static uint8 au8SerialRxBuffer[32];

    sSerPort.pu8SerialRxQueueBuffer = au8SerialRxBuffer;
    sSerPort.pu8SerialTxQueueBuffer = au8SerialTxBuffer;
    sSerPort.u32BaudRate = UART_BAUD;
    sSerPort.u16AHI_UART_RTS_LOW = 0xffff;
    sSerPort.u16AHI_UART_RTS_HIGH = 0xffff;
    sSerPort.u16SerialRxQueueSize = sizeof(au8SerialRxBuffer);
    sSerPort.u16SerialTxQueueSize = sizeof(au8SerialTxBuffer);
    sSerPort.u8SerialPort = E_AHI_UART_0;
    sSerPort.u8RX_FIFO_LEVEL = E_AHI_UART_FIFO_LEVEL_1;
    SERIAL_vInit(&sSerPort);

    sSerStream.bPutChar = SERIAL_bTxChar;
    sSerStream.u8Device = E_AHI_UART_0;
}

// ユーザ定義のイベントハンドラ
// ウィンドウズのwndProc()みたいなもん
// 比較的重めの処理を書いてもいいけどブロックしてはいけません
static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg)
{
    static uint8 nChars;
    static uint8 nRetry;
    static uint16 tRetry = T_RETRY_MIN;
    static uint8 nSample = 0;
    static uint32 tTotal = 0;

    switch(pEv->eState) {
    case E_STATE_IDLE:
        if (eEvent == E_EVENT_START_UP) {
            nChars = CHARS_MIN;
            nRetry = RETRY_MIN;

#ifdef MEASURE_VOLTAGE
            debug("nChars,nRetry,tRetry[ms],Time[ms],Min[mV],Max[mV]");

            // 1) アナログ部の電源投入
            if (!bAHI_APRegulatorEnabled()) {
                vAHI_ApConfigure(E_AHI_AP_REGULATOR_ENABLE,
                    E_AHI_AP_INT_ENABLE,
                        // DISABLE にするとアナログ部の電源断
                        // 電池駆動の場合は測定後に電源を落とすべきでしょうね
                    E_AHI_AP_SAMPLE_4,
                        // サンプル数 2,4,6,8 が選択可能
                    E_AHI_AP_CLOCKDIV_500KHZ,
                        // 周波数 250K/500K/1M/2M。500KHZが推奨だそうです。
                    E_AHI_AP_INTREF);

                // レギュレーター部安定待ち
                // 一瞬で完了するみたいなので、こんな待ち方でOK
                while(!bAHI_APRegulatorEnabled()) ;
            }

            // 2) ADC 開始
            vAHI_AdcEnable(
                    E_AHI_ADC_CONTINUOUS,
                        // E_AHI_ADC_SINGLE_SHOT １回のみ
                        // E_AHI_ADC_CONTINUOUS 連続実行
                    E_AHI_AP_INPUT_RANGE_2,
                        // E_AHI_AP_INPUT_RANGE_1 (0-1.2V)
                        // または E_AHI_AP_INPUT_RANGE_2 (0-2.4V)
                    E_AHI_ADC_SRC_VOLT
                        // E_AHI_ADC_SRC_ADC_1 (ADC1)
                        // E_AHI_ADC_SRC_ADC_2 (ADC2)
                        // E_AHI_ADC_SRC_ADC_3 (ADC3)
                        // E_AHI_ADC_SRC_ADC_4 (ADC4)
                        // E_AHI_ADC_SRC_TEMP (温度)
                        // E_AHI_ADC_SRC_VOLT (電圧)
                    );

            vAHI_AdcStartSample(); // ADC開始
#else
            debug("nChars,nRetry,tRetry[ms],Time[ms]");
#endif
            ToCoNet_Event_SetState(pEv, E_STATE_APP_WAIT_TIME);
        }
        break;
    case E_STATE_APP_WAIT_TIME:
        if (eEvent == E_EVENT_TICK_SECOND) {
            sendBroadcast(nChars, nRetry, tRetry);

            adcMax = 0;
            adcMin = 0xffffffff;

            ToCoNet_Event_SetState(pEv, E_STATE_APP_WAIT_TX);
            break;
        }
    case E_STATE_APP_WAIT_TX:
        if (eEvent == E_ORDER_KICK) {

            tTotal += t0;
            if(++nSample >= N_SAMPLING) {

                tTotal = tTotal * 10 / nSample;

#ifdef MEASURE_VOLTAGE
                // 電圧に換算
                uint32 voltMin = (adcMin * 3600) / 1023;
                uint32 voltMax = (adcMax * 3600) / 1023;

                debug("%u,%u,%u,%u,%u,%u", (uint32)nChars, (uint32)nRetry, (uint32)tRetry, tTotal, voltMin, voltMax);
#else
                debug("%u,%u,%u,%u", (uint32)nChars, (uint32)nRetry, (uint32)tRetry, tTotal);
#endif

                nChars += CHARS_STEP;
                if(nChars > CHARS_MAX){
                    nChars = CHARS_MIN;

                    nRetry += RETRY_STEP;
                    if(nRetry > RETRY_MAX) {
                        nRetry = RETRY_MIN;

                        tRetry += T_RETRY_STEP;
                        if(tRetry > T_RETRY_MAX) {

                            debug("Finished");
                            ToCoNet_Event_SetState(pEv, E_STATE_IDLE);
                            break;
                        }
                    }
                }

                nSample = 0;
                tTotal = 0;
            }
            ToCoNet_Event_SetState(pEv, E_STATE_APP_WAIT_TIME);
        }
        break;
    }
}

// 電源オンによるスタート
void cbAppColdStart(bool_t bAfterAhiInit)
{
	if (!bAfterAhiInit) {
        // 必要モジュール登録手続き
        ToCoNet_REG_MOD_ALL();
	} else {
        // SPRINTF 初期化
        SPRINTF_vInit128();

        // ToCoNet パラメータ
        sToCoNet_AppContext.u32AppId = APP_ID;
        sToCoNet_AppContext.u8Channel = CHANNEL;
        sToCoNet_AppContext.bRxOnIdle = FALSE;  // 受信しない
        sToCoNet_AppContext.u16TickHz = 1000;   // デフォの4msから1msのカウンタに切り替え

        // ユーザ定義のイベントハンドラを登録
        ToCoNet_Event_Register_State_Machine(vProcessEvCore);

		// シリアル出力用
		vSerialInit();
		ToCoNet_vDebugInit(&sSerStream);
		ToCoNet_vDebugLevel(0);

        // MAC 層開始
        ToCoNet_vMacStart();
	}
}

// スリープからの復帰
void cbAppWarmStart(bool_t bAfterAhiInit)
{
    //今回は使わない
}

// ネットワークイベント発生時
void cbToCoNet_vNwkEvent(teEvent eEvent, uint32 u32arg)
{
	switch(eEvent) {
	default:
		break;
	}
}

// パケット受信時
void cbToCoNet_vRxEvent(tsRxDataApp *pRx)
{
}

// パケット送信完了時
void cbToCoNet_vTxEvent(uint8 u8CbId, uint8 bStatus)
{
    //送信したよ！
    t0 = u32TickCount_ms - t0;
    ToCoNet_Event_Process(E_ORDER_KICK, 0, vProcessEvCore);
}

// ハードウェア割り込み発生後（遅延呼び出し）
void cbToCoNet_vHwEvent(uint32 u32DeviceId, uint32 u32ItemBitmap)
{
    //割り込みに対する処理は通常ここで行う。
	switch (u32DeviceId) {

        // ADC変換が完了した
	case E_AHI_DEVICE_ANALOGUE:;

        uint32 val = u16AHI_AdcRead();
        if(adcMax < val) adcMax = val;
        if(adcMin > val) adcMin = val;
        break;
    }
}

// ハードウェア割り込み発生時
uint8 cbToCoNet_u8HwInt(uint32 u32DeviceId, uint32 u32ItemBitmap)
{
    //割り込みで最初に呼ばれる。最短で返さないといけない。
	return FALSE;//FALSEによりcbToCoNet_vHwEvent()が呼ばれる
}

// メイン
void cbToCoNet_vMain(void)
{
}
