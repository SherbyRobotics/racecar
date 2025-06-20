/*=============================================================================================================================
Author: Marc-Antoine C. Lafrenière, Ian Lalonde
Project: racecar
Date: Jan 2024
==============================================================================================================================*/
#include "PBUtils.h"

/*
 * Constructor of the class
 * 
 * @param topics: Array of all the topics containing their type
 */ 
PBUtils::PBUtils(const Topic *topics)
{
  for (int i = 0; i < _NBS_TOPICS; ++i)
  {
    idToType[topics[i].id] = topics[i].type;
    idToMsg[topics[i].id] = topics[i].msg;
  }
}

PBUtils::~PBUtils() { }

/*
 * Convert a string to a list of PB messages
 * 
 * @param input_string: The input str to decode from
 * @param sub_msg_id: An output list of all the messages id in the full message
 * @param nbs_new_msgs: The number of sub messages in the fill message
 * 
 * @return: if the decode was successful
 */
bool PBUtils::decodePb(char* inputString, int *subMsgId, int &nbsNewMsgs)
{
  char *subMsgs[MAX_MSG_LEN];
  bool success = true;
  
  nbsNewMsgs = parseMsg(inputString, subMsgId, subMsgs);

  // Add assert nbs_new_msgs < length of sub_msg_id

  for (int i = 0; i < nbsNewMsgs; ++i)
  {
    uint8_t bufferIn[MAX_MSG_LEN];
    charsToBytes(subMsgs[i], bufferIn);

    pb_istream_t streamIn = pb_istream_from_buffer(bufferIn, strlen(subMsgs[i])/2);
    success = success && pb_decode(&streamIn, idToType[subMsgId[i]], idToMsg[subMsgId[i]]);
  }
  
  return success;
}

/*
 * Send protobufs messages to the serial port with format <id|msg;>
 * 
 * @param nbs: The numbers of id to send
 * @param ...: List of all the ids to send 
 *             ex : cant use it like this pb_send(3, POS, OBS_POS, DEBUG_ARDUINO);
 */
void PBUtils::pbSend(int nbs, ...)
{
  bool success = true;
  String toSendBuilder = "<";
  va_list idsToSend;
  va_start(idsToSend, nbs);

  for (int i = 0; i < nbs; ++i)
  {
    int id = va_arg ( idsToSend, int );
    uint8_t bufferOut[MAX_MSG_LEN];
    char toSend[10];  // The biggest byte array in the nanopb encode seems to be 10 (should be coded to not depend on it tho...)
    pb_ostream_t stream = pb_ostream_from_buffer(bufferOut, sizeof(bufferOut));
    success = success && pb_encode(&stream, idToType[id], idToMsg[id]);

    if(success)
    {
      toSendBuilder += String(id);
      toSendBuilder += "|";
      
      for(size_t j = 0; j < stream.bytes_written; j++)
      {
        sprintf (toSend, "%02X", bufferOut[j]);
        toSendBuilder += String(toSend);
      }
    }
    
    toSendBuilder += ";";
  }
  toSendBuilder += ">";
  va_end(idsToSend);

  if (success)
    Serial.print(toSendBuilder);
}

/*
 * Convert an input string to a list of PB messages and ids
 * 
 * @param in_string: The input string to parse
 * @param msg_ids: Output a list of id of all the sub messages in the full message
 * @param out_strings: Output a list of str containting the messages (without id)
 * 
 * @return: The number of submessages
 */
int PBUtils::parseMsg(char inString[], int *msgIds, char **outStrings)
{
  // Split all messages
  char *wholeMsgs[MAX_MSG_LEN];
  int ind = 0;
  char* substr = strtok(inString, ";");

  while (substr != NULL)
  {
    wholeMsgs[ind++] = substr;
    substr = strtok(NULL, ";");
  }

  // Split (id, msg) pair
  for (int i = 0; i < ind; i++)
  {
    char* id = strtok(wholeMsgs[i], "|");
    char* msg = strtok(NULL, "|");
    
    msgIds[i] = atoi(id);
    outStrings[i] = msg;
  }

  return ind;
}

/*
 * Convert a string to a list of uint
 *
 * @param in_string: The input string to convert to hex
 * @param string_values: The output of the conversion (size divided by 2 since 2 HEX is on uint8_t)
 * 
 * @return: 
 */
void PBUtils::charsToBytes(char* inString, uint8_t* stringValue)
{
  int len = strlen(inString)/2;
  for (int i = 0; i < len; ++i)
    stringValue[i] = charToHex(inString[(i*2)], inString[(i*2)+1]);
}

/*
 * Convert two char (between 00 and FF) to an HEX value 
 * 
 * @param in: Array of two char to be converted
 * 
 * @return: The decimal value
 */
uint8_t PBUtils::charToHex(char in1, char in2)
{
  uint8_t val[2];
  char in[2] = { in1, in2};

  for (int i = 0; i < 2; i++)
  {
    if (in[i] >= 'A' && in[i] <= 'F')
      val[i] = in[i] - 'A' + 10;
    else if (in[i] >= 'a' && in[i] <= 'f')
      val[i] = in[i] - 'a' + 10;
    else
      val[i] = in[i] - '0';
  }
 
  return val[0]*16 + val[1];
}
