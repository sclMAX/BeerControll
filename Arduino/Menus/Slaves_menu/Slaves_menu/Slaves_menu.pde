// Written by Dr. John Liu for a joint project with JO3RI for phi-panel
// Date: 09/18/2011, 80 years since the invasion of Japan to China, which resulted in 30 million death in 15 years.
#include <NewSoftSerial.h>
#include <avr/pgmspace.h>

PROGMEM prog_char menu_0_00[]="Slave Menu";
PROGMEM prog_char menu_0_01[]="1.Make slave";
PROGMEM prog_char menu_0_02[]="2.NR slave 1-8";
PROGMEM prog_char menu_0_03[]="3.4th IP field";
PROGMEM prog_char menu_0_04[]="4.Group # 1-8";
PROGMEM prog_char menu_0_05[]="5.Submit";
PROGMEM const char *menu_0_items[]= {menu_0_00,menu_0_01,menu_0_02,menu_0_03,menu_0_04,menu_0_05};
char out_buffer[32];
char YN_dialog[]="\eB", interactive_menu[]="\eF", long_msg[]="\eF";
char intro[]="Joint project between:\nJO3RI\n    and\n        Liudr\n";
char esc[]="Press any key to continue...";

NewSoftSerial NSS(2,3);

void setup()
{
  NSS.begin(19200);
  delay(100);
  credits();
}

void loop()
{
  main_menu();
} 

void main_menu()
{
  NSS.print("\eG"); // Start long message
  for (byte i=0;i<sizeof(menu_0_items)/sizeof(menu_0_items[0]);i++) // Print the menu from PROGMEM. The fist sizeof() on the array name will give number of bytes of the pointer array and dividing by sizeof() the first array element will give you number of elements of the array. It's a bit of pre-compiler automation. 
  {
    PROGMEM_to_NSS((char*)pgm_read_word(menu_0_items+i),out_buffer);
    if (i<sizeof(menu_0_items)-1) NSS.print('\n'); // Print new line only between items.
  }
  NSS.print('~'); // End of long message
  
  while(1) {
    if (NSS.available()) {
      char response=NSS.read();
      if (response=='1') make_slave(); 
      if (response=='2') NR_slave(); 
      if (response=='3') ip_field(); 
      if (response=='4') group_number(); 
      if (response=='5') submit();
      break; // This breaks out of the while(1) loop.
    }
  }
}

void make_slave()
{
  NSS.print('\f'); // Clear screen
  NSS.print("Making slave");
  delay(3000);
}

void NR_slave()
{
  NSS.print('\f'); // Clear screen
  NSS.print("NR slave");
  delay(3000);
}
void ip_field()
{
  long response; // Numerical value of the user response
  NSS.print('\f'); // Clear screen
  NSS.print("\e[5m~"); // Enable blinking cursor
  NSS.print("4th IP field:\n"); // Print the question
  
  response=get_int(); // Calls the get_int function to get an integer number from phi-panel
  
  NSS.print("\e[25m~"); // Disable blinking cursor
  NSS.print("\nAddress is:\n");
  NSS.print("192.168.0.");
  NSS.print(response);
  delay(3000);
}

void group_number()
{
  NSS.print('\f'); // Clear screen
  NSS.print("Enter group #:");
  while(1) {
    if (NSS.available()) {
      char response=NSS.read();
      if ((response>'0')&&(response<'9'))
      {
        NSS.print("\nSelected ");
        NSS.print(response);
        break;
      }
    } // Wait until the user presses 1-8 for group
  }
  delay(3000);
}
void submit()
{
  NSS.print('\f'); // Clear screen
  NSS.print(YN_dialog); // Display a YN dialog with the following information
  NSS.print("Submit?");
  NSS.print('~'); // End the YN dialog
  while(1) {
    if (NSS.available()) {
      char response=NSS.read();
      if (response=='Y') NSS.print("\nSubmitted!");
      else NSS.print("\nCanceled!");
      break;} // Wait until the user presses the enter to either choose yes or no on the dialog
  }
  delay(3000);
}

void credits()
{
  NSS.print("\eG"); // Start long message
  NSS.print(intro); // Print introduction as long message
  NSS.print(esc); // Print "press any key to continue"
  NSS.print('~'); // End of long message
  
  while(1) {
    if (NSS.available()) {
      char response=NSS.read();
      break;} // Wait until a key is pressed before continue
  }
}

long get_int() // Understands backspace and enter
{
  char in_char[16]; // Char buffer to store incoming characters
  int i=0;
  int response; // Numerical value of the user response
  while(1) {
    if (NSS.available()) {
      in_char[i]=NSS.read(); // Read in one character
      NSS.print(in_char[i]); // Echo key press back to the panel for the user to see
      if ((in_char[i]=='\b')&&(i>0)) i-=2; // Handles back space.
      if (in_char[i]=='\n') { // The \n represents enter key.
        in_char[i]=0; // Terminate the string with 0.
        sscanf(in_char, "%d", &response); // Get the number from the string
        break; // This breaks out of the while(1) loop.
      }
      i++;
      if (i<0) i=0;
    }
  }
  return response;
}

void PROGMEM_to_NSS(char* msg_line, char* msg_buffer) // This function simply copies a char array from PROGMEM, msg_line, to a provided buffer, msg_buffer, then print it to NSS, new soft serial object.
{
  strcpy_P(msg_buffer,msg_line); 
  NSS.print(msg_buffer);
}
