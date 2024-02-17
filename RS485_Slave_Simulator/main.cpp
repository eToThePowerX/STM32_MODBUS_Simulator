#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#define slave_id 0x02

enum res{VOLTAGE, CURRENT, REACTIVE_POWER, POWER_FACTOR};

void send(enum res r, uint16_t nBytes);
uint8_t *hex_array(float f);
uint16_t XOR(uint16_t constant, uint16_t datum);
void crc(uint8_t len, uint16_t cvalue[]);

uint8_t upperCRC, lowerCRC;

int main()
{
    //uint8_t uart_response[250];
    uint8_t uart_response[] = {0x02, 0x03, 0x01, 0x00, 0x00, 0x10, 0x45, 0xC9};
    if(uart_response[0] == slave_id && uart_response[1] == 0x03)
    {
        uint16_t sAddress = (uart_response[2] << 8) + uart_response[3];
        uint16_t nBytes = (uart_response[4] << 8) + uart_response[5];
        uint16_t vBytes[6];
        for(uint8_t i=0;i<6;i++)
        {
            vBytes[i] = uart_response[i];
        }
        crc(6,vBytes);
        if(lowerCRC == uart_response[6] && upperCRC == uart_response[7])
        {
            if(sAddress == 0x0100)
            {
                send(VOLTAGE, nBytes);
            }
            else if(sAddress == 0x0120)
            {
                send(CURRENT, nBytes);
            }
            else if(sAddress == 0x014E)
            {
                send(REACTIVE_POWER, nBytes);
            }
            else if(sAddress == 0x0132)
            {
                send(POWER_FACTOR, nBytes);
            }
        }
    }

    return 0;
}

void send(enum res r, uint16_t nBytes)
{
    uint8_t *arr;

    float VP[] = {230.47, 240.15, 0.0};
    float AMP[] = {2.34, 1.3, 5.0};
    float RAP[] = {1.78,2.65,9.25};
    float PF[] = {1.0, 0.99, 0.95, 0.87};

    uint8_t sz = 5 + (nBytes * 2);
    uint8_t slave_send[sz];
    slave_send[0] = slave_id;
    slave_send[1] = 0x03;
    slave_send[2] = nBytes * 2;

    uint8_t k = 3;
    for(uint8_t j=0;j<(nBytes/2);j++)
    {
        if(r == VOLTAGE)
        {
            if(j>2)
            {
                arr = hex_array(0.0);
            }
            else
            {
                arr = hex_array(VP[j]);
            }
        }
        else if(r == CURRENT)
        {
            if(j>2)
            {
                arr = hex_array(0.0);
            }
            else
            {
                arr = hex_array(AMP[j]);
            }
        }
        else if(r == REACTIVE_POWER)
        {
            if(j>2)
            {
                arr = hex_array(0.0);
            }
            else
            {
                arr = hex_array(RAP[j]);
            }
        }
        else if(r == POWER_FACTOR)
        {
            if(j>3)
            {
                arr = hex_array(0.0);
            }
            else
            {
                arr = hex_array(PF[j]);
            }
        }

        for(uint8_t i=0;i<4;i++)
        {
            slave_send[i+k] = arr[i];
        }
        k += 4;
    }

    uint16_t vBytes[(nBytes * 2) + 3];
    for(uint8_t i=0;i<((nBytes * 2)+3);i++)
    {
        vBytes[i] = slave_send[i];
    }

    if(nBytes > 6)
    {
        while(k < (nBytes+3))
        {
            slave_send[k++] = 0x00;
        }
    }
    crc(((nBytes*2)+3),vBytes);
    slave_send[k++] = lowerCRC;
    slave_send[k++] = upperCRC;

    printf("\nSlave Response\n");
    for(uint8_t i=0;i<sz;i++)
    {
        printf(" 0x%X ",slave_send[i]);
    }
}

uint8_t *hex_array(float f)
{
    static uint8_t reply[4];
    uint32_t num = *((uint32_t*)&f);
    //printf("0x%X\n",num);
    reply[0] = (num & 0x0000FF00) >> 8;
    reply[1] = (num & 0x000000FF);
    reply[2] = (num & 0xFF000000) >> 24;
    reply[3] = (num & 0x00FF0000) >> 16;
    return reply;
}

uint16_t XOR(uint16_t constant, uint16_t datum)
{
  uint8_t i;
  uint16_t polynomial = 0xA001;
  datum = constant ^ datum;
  for(i=0;i<8;i++)
  {
    if((datum & (0x0001)) == 0)
    {
      datum = datum >> 1;
      datum = datum & (0x7FFF);
    }
    else
    {
      datum = datum >> 1;
      datum = datum & (0x7FFF);
      datum = datum ^ polynomial;
    }
  }
  return datum;
}

void crc(uint8_t len, uint16_t cvalue[])
{
	uint16_t finalCRC, j;

  for(j=0;j<len;j++)
  {
    if(j==0)
    {
      cvalue[j] = XOR(0xFFFF,cvalue[j]);
    }
    else
    {
      cvalue[j] = XOR(cvalue[j-1],cvalue[j]);
      finalCRC = cvalue[j];
    }
    if(j==len-1)
    {
      upperCRC = (finalCRC & (0xFF00)) >> 8;
      lowerCRC = finalCRC & (0x00FF);
    }
  }
}
