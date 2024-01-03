
#include "main.h"

/******************************************************************************/

static char hbuf[128];
static int hblen = 0;
#define PROMPT "rv"

int gets(char *buf, int len)
{
	int n, ch, esc;

	printk(PROMPT "> ");
	n = 0;
	esc = 0;
	while(1){
		while((ch=_getc(1000))<0);
		if(ch==0x0d || ch==0x0a){
			break;
		}

		if(esc==0 && ch==0x1b){
			esc = 1;
			continue;
		}
		if(esc==1){
			if(ch==0x5b){
				esc = 2;
			}else{
				printk("<ESC1 %02x>", ch);
				esc = 0;
			}
			continue;
		}
		if(esc==2){
			esc = 0;
			if(ch==0x41 || ch==0x43){
				// UP:0x41  RIGHT:0x43
				if(hblen){
					int sn = n;
					n = hblen;
					memcpy(buf, hbuf, hblen);
					printk("\r" PROMPT "> %s", hbuf);
					while(hblen<sn){
						printk(" ");
						hblen += 1;
					}
					if(ch==0x43){
						break;
					}
				}
			}else{
				printk("<ESC2 %02x>", ch);
			}
		}else if(ch==0x08){
			if(n){
				n -= 1;
				printk("\b \b");
			}
		}else if(ch>=0x20){
			printk("%c", ch);
			buf[n] = ch;
			n++;
			if(n==len)
				break;
		}else{
			printk("{%02x}", ch);
		}
	}

	if(n){
		memcpy(hbuf, buf, n);
		hbuf[n] = 0;
		hblen = n;
	}

	printk("\r\n");
	buf[n] = 0;
	return n;
}


static int arg_base = 16;

int str2hex(char *str, int *hex)
{
	int n, p, ch, base;

	base = arg_base;
	n = 0;
	p = 0;
	hex[0] = 0;
	while(1){
		ch = *str++;
		if(ch==' '){
			if(p){
				n++;
				hex[n] = 0;
				p = 0;
			}
			continue;
		}else if((ch=='0' && *str=='x')){
			str += 1;
			base = 16;
		}else if((ch>='0' && ch<='9')){
			ch -= '0';
		}else if((base==16 && ch>='a' && ch<='f')){
			ch = ch-'a'+10;
		}else if((base==16 && ch>='A' && ch<='F')){
			ch = ch-'A'+10;
		}else{
			if(p)
				n++;
			return n;
		}
		hex[n] = hex[n]*base + ch;
		p++;
	}
}


/**********************************************************/

static int dump_width = 4;
static int dump_addr = 0;

void dump(int argc, int *args, int width)
{
	int p, addr, len;
	u32 d;

	addr = (argc>=1)? args[0] : dump_addr;
	len  = (argc>=2)? args[1] : 256;
	if(width==0)
		width = dump_width;

	p = 0;
	while(p<len){
		if((p%16)==0){
			printk("\n%08x:  ", addr);
		}else if((p%8)==0){
			printk("- ");
		}

		if(width==1){
			d = *(u8*)(addr);
			printk("%02x ", d);
		}else if(width==2){
			d = *(u16*)(addr);
			printk("%04x ", d);
		}else{
			d = *(u32*)(addr);
			printk("%08x ", d);
		}
		p += width;
		addr += width;
	}
	printk("\n");

	dump_width = width;
	dump_addr = addr;
}



/******************************************************************************/


#define CMD_START if(0){}
#define CMD(x) else if(strcmp(cmd, #x)==0)

void simple_shell(void)
{
	char cmd[128];
	int argc, arg[4];

	dump_addr = 0x08000000;

	while(1){
		gets(cmd, 128);
		char *sp = strchr(cmd, ' ');
		if(sp){
			*sp = 0;
			sp += 1;
			argc = str2hex(sp, arg);
		}else{
			argc = 0;
		}

		CMD_START
		CMD(base16) { arg_base = 16; }
		CMD(base10) { arg_base = 10; }
		CMD(rb){ printk("%08x: %02x\n", arg[0], *(u8*)arg[0]);  }
		CMD(rw){ printk("%08x: %04x\n", arg[0], *(u16*)arg[0]); }
		CMD(rd){ printk("%08x: %08x\n", arg[0], *(u32*)arg[0]);	}
		CMD(wb){ printk("%08x= %02x\n", arg[0], (u8)arg[1]);  *(u8*)arg[0]  = arg[1]; }
		CMD(ww){ printk("%08x= %04x\n", arg[0], (u16)arg[1]); *(u16*)arg[0] = arg[1]; }
		CMD(wd){ printk("%08x= %08x\n", arg[0], (u32)arg[1]); *(u32*)arg[0] = arg[1]; }
		CMD(db){ dump(argc, arg, 1); }
		CMD(dw){ dump(argc, arg, 2); }
		CMD(dd){ dump(argc, arg, 4); }
		CMD(d) { dump(argc, arg, 0); }
		CMD(go){ void (*go)(void) = (void(*)(void))arg[0]; go(); }
		
		CMD(memset){
			if(argc>=3){
				memset((void*)arg[0], arg[1], arg[2]);
			}else{
				printk("Wrong args!\n");
			}
		}
		CMD(memcpy){
			if(argc>=3){
				memcpy((void*)arg[0], (void*)arg[1], arg[2]);
			}else{
				printk("Wrong args!\n");
			}
		}


		CMD(mt){
			int cnt=0;
			while(cnt<51){
				printk("cnt: %d\n", cnt);
				gpio_set(0, 8, cnt&1);
				cnt += 1;
				udelay(1000000);
			}
		}

		CMD(tt){
		    u64 mc = get_mcycle();
		    printk("mcycle: %08x_%08x\n", (u32)(mc>>32), (u32)mc);
		}

		CMD(usbd){
		    void usb_dc_user_init(void);
		    usb_dc_user_init();
		}

#ifdef RUN_COREMARK
		CMD(cmk){
			void cmain(void);
			cmain();
		}
#endif

		CMD(q){
			break;
		}
	}
}

