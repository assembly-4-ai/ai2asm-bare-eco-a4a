; boot_defs_temp.inc: Consolidated Definitions for UEFI Bootloader

%ifndef BOOT_DEFS_TEMP_INC ; Guard for boot_defs_temp.inc
%define BOOT_DEFS_TEMP_INC

; ===========================================
; --- Core System Definitions ---
; ===========================================
%define IDT_MAX_ENTRIES         256
%define KEY_BUFFER_SIZE         256
%define CODE64_SEL              0x18
%define DATA64_SEL              0x20

; --- Page Sizes ---
%define PAGE_SIZE_4K            4096
%define PAGE_SIZE_2M            (2 * 1024 * 1024)

; --- Page Table Entry Flags ---
%define PTE_PRESENT             (1 << 0)
%define PTE_WRITABLE            (1 << 1)
%define PTE_USER                (1 << 2)
%define PTE_WT                  (1 << 3)
%define PTE_CD                  (1 << 4)
%define PTE_ACCESSED            (1 << 5)
%define PTE_DIRTY               (1 << 6)
%define PTE_PS                  (1 << 7)
%define PTE_GLOBAL              (1 << 8)
%define PTE_XD                  (1 << 63)

; --- Standard PTE Flags ---
%define KERNEL_PTE_FLAGS        (PTE_PRESENT | PTE_WRITABLE)
%define DATA_PTE_FLAGS          (PTE_PRESENT | PTE_WRITABLE | PTE_XD)

; ===========================================
; --- Legacy Hardware Definitions (If Used) ---
; ===========================================
%define PIC1_COMMAND            0x20
%define PIC1_DATA               0x21
%define PIC2_COMMAND            0xA0
%define PIC2_DATA               0xA1
%define PIC_EOI                 0x20
%define PIC1_IRQ_START          0x20
%define PIC2_IRQ_START          0x28

%define KB_DATA_PORT            0x60
%define KB_STATUS_PORT          0x64
%define KB_IRQ                  1
%define SC_LSHIFT_MAKE          0x2A
%define SC_LSHIFT_BREAK         0xAA
%define SC_RSHIFT_MAKE          0x36
%define SC_RSHIFT_BREAK         0xB6
%define SC_CAPSLOCK_MAKE        0x3A

%define PCI_CONFIG_ADDRESS      0xCF8
%define PCI_CONFIG_DATA         0xCFC
%define PCI_CLASS_MASS_STORAGE  0x01
%define PCI_SUBCLASS_SATA       0x06
%define PCI_PROGIF_AHCI         0x01

; ===========================================
; --- FAT32 Driver Definitions ---
; ===========================================
%define FAT_ERR_OK              0; %define FAT_ERR_NOT_INIT        1
%define FAT_ERR_DISK_ERROR      2; %define FAT_ERR_NOT_FOUND       3
%define FAT_ERR_BAD_CLUSTER     4; %define FAT_ERR_FAT_READ        5
%define FAT_ERR_EOF             6; %define FAT_ERR_BUFFER_SMALL    7
%define FAT_ERR_NOT_FAT32       8; %define FAT_ERR_BAD_BPB         9
%define FAT_ERR_INVALID_PARAM   10; %define FAT_ERR_NO_FREE_CLUSTER 11
%define FAT_ERR_NOT_IMPLEMENTED 12; %define FAT_ERR_FILE_EXISTS     13
%define FAT_ERR_DIR_FULL        14; %define FAT_ERR_GPT_ERROR       15

%define FAT_BPB_BytesPerSector    11; %define FAT_BPB_SectorsPerCluster 13
%define FAT_BPB_ReservedSectors   14; %define FAT_BPB_NumberOfFATs      16
%define FAT32_BPB_SectorsPerFAT32 36; %define FAT32_BPB_RootCluster     44
%define FAT32_BPB_FSInfo          48; %define FAT32_BPB_FSType          82

%define FAT_DIRENT_Name         0 ; %define FAT_DIRENT_Attributes   11; %define FAT_DIRENT_NTRes        12
%define FAT_DIRENT_CrtTimeTenth 13; %define FAT_DIRENT_CrtTime      14; %define FAT_DIRENT_CrtDate      16
%define FAT_DIRENT_LstAccDate   18; %define FAT_DIRENT_FstClusHI    20; %define FAT_DIRENT_WrtTime      22
%define FAT_DIRENT_WrtDate      24; %define FAT_DIRENT_FstClusLO    26; %define FAT_DIRENT_FileSize     28
%define FAT_DIRENT_SIZE         32

%define FAT_ATTR_READ_ONLY      0x01; %define FAT_ATTR_HIDDEN         0x02; %define FAT_ATTR_SYSTEM         0x04
%define FAT_ATTR_VOLUME_ID      0x08; %define FAT_ATTR_DIRECTORY      0x10; %define FAT_ATTR_ARCHIVE        0x20
%define FAT_ATTR_LONG_NAME      0x0F; %define FAT_ATTR_LFN_MASK       0x3F

%define FAT32_CLUSTER_FREE      0x00000000; %define FAT32_CLUSTER_RESERVED  0x00000001
%define FAT32_CLUSTER_MIN_VALID 0x00000002; %define FAT32_CLUSTER_MAX_VALID 0x0FFFFFF6
%define FAT32_CLUSTER_BAD       0x0FFFFFF7; %define FAT32_CLUSTER_EOF_MIN   0x0FFFFFF8
%define FAT32_CLUSTER_EOF_MAX   0x0FFFFFFF

; --- FAT32 BPB Offsets ---
%define FAT_BPB_SectorsPerCluster 13 ; BYTE
%define FAT_BPB_NumberOfFATs      16 ; BYTE
%define FAT32_BPB_RootCluster     44 ; DWORD

; --- FAT32 Cluster Constants ---
%define FAT32_CLUSTER_EOF_MIN   0x0FFFFFF8 ; Minimum value for EOF

; --- FAT32 Error Codes ---
%define FAT_ERR_FAT_READ        5
%define FAT_ERR_NOT_INIT        1
%define FAT_ERR_NOT_FOUND       3

; --- FAT32 Directory Entry Offsets ---
%define FAT_DIRENT_Attributes   11 ; BYTE

%define GPT_HEADER_SIGNATURE    0x5452415020494645; %define GPT_PMBR_LBA            0; %define GPT_HEADER_OFFSET       1
%define GPT_PARTITION_ENTRY_SIZE 128; %define GPT_HDR_SIGNATURE       0x00; %define GPT_HDR_REVISION        0x08
%define GPT_HDR_HDR_SIZE        0x0C; %define GPT_HDR_CRC32           0x10; %define GPT_HDR_CURRENT_LBA     0x18
%define GPT_HDR_BACKUP_LBA      0x20; %define GPT_HDR_FIRST_USABLE    0x28; %define GPT_HDR_LAST_USABLE     0x30
%define GPT_HDR_DISK_GUID       0x38; %define GPT_HDR_PART_ENTRY_LBA  0x48; %define GPT_HDR_NUM_PART_ENTRIES 0x50
%define GPT_HDR_PART_ENTRY_SIZE 0x54; %define GPT_HDR_PART_CRC32      0x58; %define GPT_PTE_TYPE_GUID       0x00
%define GPT_PTE_UNIQUE_GUID     0x10; %define GPT_PTE_FIRST_LBA       0x20; %define GPT_PTE_LAST_LBA        0x28
%define GPT_PTE_ATTRIBUTES      0x30; %define GPT_PTE_NAME            0x38

GPT_GUID_FAT32_PART: dd 0xC12A7328; dw 0xF81F, 0x11D2; db 0xBA, 0x4B, 0x00, 0xA0, 0xC9, 0x3E, 0xC9, 0x3B

EFI_GRAPHICS_OUTPUT_PROTOCOL_GUID: dd 0x9042A9DE; dw 0x23DC, 0x4A38; db 0x96, 0xFB, 0x7A, 0xDE, 0xD0, 0x80, 0x51, 0x6A
EFI_LOADED_IMAGE_PROTOCOL_GUID: dd 0x964E5B21; dw 0x6459, 0x11D2; db 0x8E, 0x39, 0x00, 0xA0, 0xC9, 0x69, 0x72, 0x3B


; ===========================================
; --- UEFI Specific Definitions ---
; ===========================================
%define EFI_SUCCESS             0; %define EFI_LOAD_ERROR          0x8000000000000001
%define EFI_INVALID_PARAMETER   0x8000000000000002; %define EFI_UNSUPPORTED         0x8000000000000003
%define EFI_BAD_BUFFER_SIZE     0x8000000000000004; %define EFI_BUFFER_TOO_SMALL    0x8000000000000005
%define EFI_NOT_READY           0x8000000000000006; %define EFI_DEVICE_ERROR        0x8000000000000007
%define EFI_WRITE_PROTECTED     0x8000000000000008; %define EFI_OUT_OF_RESOURCES    0x8000000000000009
%define EFI_NOT_FOUND           0x800000000000000E
%define EFI_SYSTEM_TABLE_SIGNATURE 0x5453595320494249
%define EFI_BOOT_SERVICES_SIGNATURE 0x56524553544f4f42
%define EfiConventionalMemory       7

;Offsets used for memmory allocations

%define OFFSET_ST_SIGNATURE        0x00; %define OFFSET_ST_CONOUT           0x30
%define OFFSET_ST_BOOTSERVICES     0x60
%define OFFSET_BS_SIGNATURE        0x00; %define OFFSET_BS_GETMEMORYMAP     0x28
%define OFFSET_BS_ALLOCATEPAGES    0x30; %define OFFSET_BS_FREEPAGES        0x38
%define OFFSET_BS_HANDLEPROTOCOL   0x40; %define OFFSET_BS_EXITBOOTSERVICES   0x50
%define OFFSET_BS_LOCATEPROTOCOL   0x70
%define OFFSET_BS_GETMEMORYMAP     0x28 ; Index 5 * 8 bytes
%define OFFSET_CONOUT_OUTPUTSTRING 0x08
%define OFFSET_GOP_MODE            0x20; %define OFFSET_GOP_MODE_INFO       0x08
%define OFFSET_GOP_MODE_FBBASE     0x18; %define OFFSET_GOP_MODE_FBSIZE     0x20
%define OFFSET_GOP_INFO_HRES       0x04; %define OFFSET_GOP_INFO_VRES       0x08
%define OFFSET_GOP_INFO_PIXELFMT   0x0C; %define OFFSET_GOP_INFO_PIXELSPERSCANLINE 0x14
%define OFFSET_LOADED_IMAGE_IMAGESIZE   0x48
%define OFFSET_GOP_MODE_FBSIZE     0x20
%define OFFSET_GOP_MODE_INFO       0x08
%define OFFSET_GOP_INFO_VRES       0x08
%define OFFSET_GOP_INFO_PIXELSPERSCANLINE 0x14
%define AllocateAnyPages            0
%define PixelBlueGreenRedReserved8BitPerColor 1
%define OFFSET_LOADED_IMAGE_IMAGEBASE   0x40; %define OFFSET_LOADED_IMAGE_IMAGESIZE   0x48
%define EfiLoaderData               2; %define EfiReservedMemoryType       0; %define EfiLoaderCode               1
%define EfiBootServicesCode         3; %define EfiBootServicesData         4; %define EfiRuntimeServicesCode      5
%define EfiRuntimeServicesData      6; %define EfiConventionalMemory       7; %define EfiUnusableMemory           8
%define EfiACPIReclaimMemory        9; %define EfiACPIMemoryNVS            10; %define EfiMemoryMappedIO           11
%define EfiMemoryMappedIOPortSpace  12; %define EfiPalCode                  13; %define EfiPersistentMemory         14
%define EFI_MEMORY_WB               (1 << 3); %define EFI_MEMORY_RUNTIME      (1 << 63)
%define EFI_UNSUPPORTED             0x8000000000000003
%define OFFSET_ST_CONOUT            0x30 ; Pointer to EFI_SIMPLE_TEXT_OUTPUT_PROTOCOL
%define OFFSET_BS_EXITBOOTSERVICES  0x50 ; Index 10 * 8 bytes

struc EFI_MEMORY_DESCRIPTOR
    .Type           resd 1  ; Offset 0
    .Padding        resd 1  ; Offset 4
    .PhysicalStart  resq 1  ; Offset 8  <--- Definition needed
    .VirtualStart   resq 1  ; Offset 16
    .NumberOfPages  resq 1  ; Offset 24 <--- Definition needed
    .Attribute      resq 1  ; Offset 32
endstruc


; ===========================================
; --- AHCI Definitions ---
; ===========================================

; --- AHCI Generic Host Control (GHCR) Offsets (Relative to AHCI Base VA) ---
%define AHCI_CAP        0x00; %define AHCI_GHC        0x04; %define AHCI_IS         0x08; %define AHCI_PI         0x0C
%define AHCI_VS         0x10; %define AHCI_CCC_CTL    0x14; %define AHCI_CCC_PORTS  0x18; %define AHCI_EM_LOC     0x1C
%define AHCI_EM_CTL     0x20; %define AHCI_CAP2       0x24; %define AHCI_BOHC       0x28


; --- AHCI_GHC Bits ---
%define AHCI_GHC_AE     (1 << 31); %define AHCI_GHC_IE     (1 << 1); %define AHCI_GHC_HR     (1 << 0)

; --- AHCI_PxCMD Bits ---
%define AHCI_PxCMD_ST   (1 << 0); %define AHCI_PxCMD_FRE  (1 << 4); %define AHCI_PxCMD_FR   (1 << 14); %define AHCI_PxCMD_CR   (1 << 15)
%define AHCI_PxCMD_ICC_ACTIVE (1 << 28)

; --- AHCI_PxTFD Bits (Task File Data) ---
%define AHCI_PxTFD_BSY  (1 << 7); %define AHCI_PxTFD_DRQ  (1 << 3); %define AHCI_PxTFD_ERR  (1 << 0)

; --- AHCI_PxIS Bits (Interrupt Status) ---
%define AHCI_PxIS_TFES  (1 << 30); %define AHCI_PxIS_DHRS  (1 << 0); %define AHCI_PxIS_PSS   (1 << 1); %define AHCI_PxIS_DSS   (1 << 2)
%define AHCI_PxIS_SDBS  (1 << 3); %define AHCI_PxIS_UFS   (1 << 4); %define AHCI_PxIS_DPS   (1 << 5)

; --- AHCI_PxIE Bits (Interrupt Enable) ---
%define AHCI_PxIE_TFEE  (1 << 30); %define AHCI_PxIE_DHRE  (1 << 0); %define AHCI_PxIE_PSE   (1 << 1); %define AHCI_PxIE_DSE   (1 << 2)
%define AHCI_PxIE_SDBE  (1 << 3); %define AHCI_PxIE_UFE   (1 << 4); %define AHCI_PxIE_DPE   (1 << 5)

%define AHCI_PxIE_DHRE  (1 << 0)  ; Device to Host Register FIS Int Enable
%define AHCI_PxIE_TFEE  (1 << 30) ; Task File Error Int Enable (Already added, verify)
%define AHCI_PxIE_DPE   (1 << 5)  ; Descriptor Processed Int Enable (Already added, verify)

; --- AHCI Command Header Structure (32 bytes, in Command List) ---
struc AHCI_COMMAND_HEADER
    .Flags      resw 1; .PRDTL      resw 1; .PRDBC      resd 1
    .CTBA       resd 1; .CTBAU      resd 1; .Reserved   resd 4
endstruc
%define AHCI_COMMAND_HEADER_SIZE 32

; --- AHCI Command Header Fields (Derived from structure for direct use) ---
%define AHCI_CH_FLAGS           0x00 ; Offset for Flags+PRDTL word
%define AHCI_CH_PRDBC           0x04 ; Offset for PRDBC dword
%define AHCI_CH_CTBA            0x08 ; Offset for CTBA dword
%define AHCI_CH_CTBAU           0x0C ; Offset for CTBAU dword
%define AHCI_PRDT_ENTRY_SIZE 16
%define AHCI_PRDT_DBA           0x00; %define AHCI_PRDT_DBAU          0x04
%define AHCI_PRDT_DBC           0x0C; %define AHCI_PRDT_DBC_I         (1 << 31)
%define AHCI_PRDT_MAX_SIZE_PER_ENTRY (1 << 22)


; AHCI_CH_FLAGS bits (Low word of DW0)
%define AHCI_CH_FLAGS_CFL_MASK  0x1F; %define AHCI_CH_FLAGS_WRITE     (1 << 6)
%define AHCI_CH_FLAGS_PRDTL_SHIFT 16

; --- AHCI Command Table Structure ---
%define AHCI_CT_CFIS_OFFSET     0x00; %define AHCI_CT_PRDT_OFFSET     0x80

; --- AHCI PRDT Entry Structure ---
struc AHCI_PRDT_ENTRY
    .DBA        resd 1; .DBAU       resd 1; .Reserved   resd 1; .DBC        resd 1
endstruc
%define AHCI_PRDT_ENTRY_SIZE 16
; --- AHCI PRDT Entry Fields (Derived) ---
%define AHCI_PRDT_DBA           0x00
%define AHCI_PRDT_DBAU          0x04
%define AHCI_PRDT_DBC           0x0C
%define AHCI_PRDT_DBC_I         (1 << 31)
%define AHCI_PRDT_MAX_SIZE_PER_ENTRY (1 << 22)

; --- FIS Types ---
%define AHCI_FIS_TYPE_REG_H2D   0x27

; --- Register FIS Fields (H2D, Type 0x27, 20 bytes) ---
struc AHCI_FIS_REG_H2D
    .fis_type       resb 1; .pm_port_flags  resb 1; .command        resb 1; .features_l     resb 1
    .lba0           resb 1; .lba1           resb 1; .lba2           resb 1; .device         resb 1
    .lba3           resb 1; .lba4           resb 1; .lba5           resb 1; .features_h     resb 1
    .count_l        resb 1; .count_h        resb 1; .icc            resb 1; .control        resb 1
    .reserved       resd 1
endstruc

%define AHCI_FIS_REG_H2D_SIZE 20
; --- AHCI H2D FIS Fields (Derived) ---
%define AHCI_FIS_TYPE           0x00
%define AHCI_FIS_PM_PORT        0x01
%define AHCI_FIS_COMMAND        0x02
%define AHCI_FIS_FEATURES_L     0x03
%define AHCI_FIS_LBA0           0x04
%define AHCI_FIS_LBA1           0x05
%define AHCI_FIS_LBA2           0x06
%define AHCI_FIS_DEVICE         0x07
%define AHCI_FIS_LBA3           0x08
%define AHCI_FIS_LBA4           0x09
%define AHCI_FIS_LBA5           0x0A
%define AHCI_FIS_FEATURES_H     0x0B
%define AHCI_FIS_COUNT_L        0x0C
%define AHCI_FIS_COUNT_H        0x0D
%define AHCI_FIS_ICC            0x0E
%define AHCI_FIS_CONTROL        0x0F

; --- ATA Command Codes ---
%define ATA_CMD_READ_DMA_EXT    0x25; %define ATA_CMD_WRITE_DMA_EXT   0x35
%define ATA_CMD_CACHE_FLUSH_EXT 0xEA; %define ATA_CMD_IDENTIFY        0xEC

; --- Misc AHCI ---
%define AHCI_CMD_SLOTS          32
%define AHCI_CMD_LIST_SIZE      (AHCI_CMD_SLOTS * AHCI_COMMAND_HEADER_SIZE)
%define AHCI_FIS_RX_SIZE        256
%define AHCI_CMD_TABLE_SIZE     8192 ; Example size for one command table
%define AHCI_PORT_REGS_SIZE     0x80
%define AHCI_IRQ_FLAG_COMPLETE  (1 << 0)
%define AHCI_IRQ_FLAG_ERROR     (1 << 1)
%define AHCI_CMD_TIMEOUT_LOOPS  500000000 ; Renamed from AHCI_CMD_TIMEOUT


; --- AHCI Port Registers Offsets ---
%define AHCI_PxCLB      0x00; %define AHCI_PxCLBU     0x04; %define AHCI_PxFB       0x08; %define AHCI_PxFBU      0x0C
%define AHCI_PxIS       0x10; %define AHCI_PxIE       0x14; %define AHCI_PxCMD      0x18; %define AHCI_PxTFD      0x20
%define AHCI_PxSIG      0x24; %define AHCI_PxSSTS     0x28; %define AHCI_PxSCTL     0x2C; %define AHCI_PxSERR     0x30
%define AHCI_PxSACT     0x34; %define AHCI_PxCI       0x38; %define AHCI_PxSNTF     0x3C; %define AHCI_PxFBS      0x40


; --- AHCI Command Table ---
%define AHCI_CT_CFIS_OFFSET     0x00; %define AHCI_CT_PRDT_OFFSET     0x80

; --- AHCI PRDT Entry ---
struc AHCI_PRDT_ENTRY
    .DBA        resd 1; .DBAU       resd 1; .Reserved   resd 1; .DBC        resd 1
endstruc

%define AHCI_PRDT_ENTRY_SIZE 16
%define AHCI_PRDT_DBA           0x00; %define AHCI_PRDT_DBAU          0x04
%define AHCI_PRDT_DBC           0x0C; %define AHCI_PRDT_DBC_I         (1 << 31)
%define AHCI_PRDT_MAX_SIZE_PER_ENTRY (1 << 22)

; --- ADD or VERIFY these definitions in boot_defs_temp.inc ---

; --- AHCI Generic Host Control (GHCR) Offsets ---
%define AHCI_GHC        0x04
%define AHCI_IS         0x08 ; Needed for global interrupt status checks
%define AHCI_PI         0x0C    ; Ports Implemented

; --- AHCI Port Registers Offsets ---
%define AHCI_PxCLBU     0x04
%define AHCI_PxFB       0x08
%define AHCI_PxFBU      0x0C
%define AHCI_PxIE       0x14
%define AHCI_PxCMD      0x18
%define AHCI_PxTFD      0x20
%define AHCI_PxSERR     0x30
%define AHCI_PxCI       0x38

; --- AHCI_GHC Bits ---
%define AHCI_GHC_IE     (1 << 1)

; --- AHCI_PxCMD Bits ---
%define AHCI_PxCMD_FRE  (1 << 4)
%define AHCI_PxCMD_FR   (1 << 14)
%define AHCI_PxCMD_CR   (1 << 15) ; Needed for waiting on ST/FRE clear

; --- AHCI_PxIE Bits ---
%define AHCI_PxIE_DPE   (1 << 5) ; Descriptor Processed Interrupt Enable

; --- AHCI Command Header Fields ---
%define AHCI_CH_FLAGS_WRITE     (1 << 6)

; --- AHCI Command Table Fields ---
%define AHCI_CT_PRDT_OFFSET     0x80

; --- AHCI PRDT Entry Fields ---
%define AHCI_PRDT_DBC_I         (1 << 31) ; Needed for 'or rax, AHCI_PRDT_DBC_I' fix

; --- Misc ---
; No definition for 'ahci_done_msg' here - that's data for ahci.asm

; --- FIS Types ---
%define AHCI_FIS_TYPE_REG_H2D   0x27

; --- ADD THESE DEFINITIONS TO boot_defs_temp.inc ---
; --- This is to satisfy ahci.asm assembly errors. ---
; --- Review carefully for proper memory allocation vs. constant definition ---

; --- AHCI Generic Host Control (GHCR) Offsets ---
%define AHCI_GHC        0x04
%define AHCI_PI         0x0C    ; Ports Implemented
%define AHCI_IS         0x08    ; Interrupt Status

; --- AHCI Port Registers Offsets ---
%define AHCI_PxCLBU     0x04
%define AHCI_PxFB       0x08
%define AHCI_PxFBU      0x0C
%define AHCI_PxIE       0x14
%define AHCI_PxCMD      0x18
%define AHCI_PxTFD      0x20
%define AHCI_PxSERR     0x30
%define AHCI_PxCI       0x38

; --- AHCI_GHC Bits ---
%define AHCI_GHC_IE     (1 << 1)

; --- AHCI_PxCMD Bits ---
%define AHCI_PxCMD_FRE  (1 << 4)
%define AHCI_PxCMD_FR   (1 << 14)
%define AHCI_PxCMD_CR   (1 << 15) ; Needed for waiting on ST/FRE clear

; --- AHCI_PxIE Bits ---
%define AHCI_PxIE_DPE   (1 << 5)  ; Descriptor Processed Interrupt Enable
%define AHCI_PxIE_DHRE  (1 << 0)  ; Device to Host Register FIS Int Enable
%define AHCI_PxIE_TFEE  (1 << 30) ; Task File Error Int Enable

; --- AHCI Command Header Fields ---
%define AHCI_CH_FLAGS_WRITE     (1 << 6)
; AHCI_CH_FLAGS would be an offset (0x00) into the Command Header structure,
; not a bitmask itself. If used as a bitmask, it's likely an error in ahci.asm.
; For now, to satisfy the symbol:
%define AHCI_CH_FLAGS           0x00 ; Offset for Flags+PRDTL word

%define AHCI_CH_PRDBC           0x04
%define AHCI_CH_CTBA            0x08
%define AHCI_CH_CTBAU           0x0C


; --- AHCI Command Table Fields ---
%define AHCI_CT_PRDT_OFFSET     0x80

; --- AHCI PRDT Entry Fields ---
%define AHCI_PRDT_DBA           0x00
%define AHCI_PRDT_DBAU          0x04
%define AHCI_PRDT_DBC           0x0C
%define AHCI_PRDT_DBC_I         (1 << 31)

; --- FIS Types & Fields ---
%define AHCI_FIS_TYPE           0x00  ; Offset for FIS Type in a FIS structure
%define AHCI_FIS_PM_PORT        0x01  ; Offset
%define AHCI_FIS_COMMAND        0x02  ; Offset
%define AHCI_FIS_LBA0           0x04  ; Offset
%define AHCI_FIS_DEVICE         0x07  ; Offset
%define AHCI_FIS_COUNT_L        0x0C  ; Offset

; --- ATA Command Codes ---
%define ATA_CMD_WRITE_DMA_EXT   0x35

; --- Misc AHCI ---
%define AHCI_CMD_LIST_SIZE      (32 * 32) ; (AHCI_CMD_SLOTS * AHCI_COMMAND_HEADER_SIZE)
%define AHCI_CMD_TABLE_SIZE     8192
%define AHCI_PORT_REGS_SIZE     0x80
%define AHCI_IRQ_FLAG_ERROR     (1 << 1)
%define AHCI_CMD_TIMEOUT_LOOPS  500000000 ; Replaced AHCI_CMD_TIMEOUT

; --- Placeholder for data defined label (SHOULD BE IN ahci.asm .data section) ---
; This will allow assembly but might cause linker errors or runtime issues
; if ahci.asm expects this to be a specific memory location.
; %define ahci_done_msg 0 ; Placeholder - REMOVE THIS if ahci_done_msg is data

; --- END OF ADDITIONS for ahci.asm errors ---


; --- ATA Command Codes ---
%define ATA_CMD_READ_DMA_EXT    0x25;
%define ATA_CMD_WRITE_DMA_EXT   0x35
%define ATA_CMD_CACHE_FLUSH_EXT 0xEA;
%define ATA_CMD_IDENTIFY        0xEC
%define AHCI_CMD_TIMEOUT_LOOPS  500000000 ; Renamed from AHCI_CMD_TIMEOUT


; ===========================================
; --- APIC Definitions ---
; ===========================================
; --- Dependencies for apic.asm ---
%define PTE_PRESENT             (1 << 0)
%define PTE_WRITABLE            (1 << 1)
%define PTE_XD                  (1 << 63)
%define PTE_CD                  (1 << 4)  ; Cache Disable flag needed for MMIO mapping

; --- PIC Constants (for disable_pic) ---
%define PIC1_DATA               0x21
%define PIC2_DATA               0xA1

; --- Keyboard Constant (for ioapic_write_rte example) ---
%define KB_IRQ                  1

; --- IRQ Vector Base (for ioapic_write_rte example) ---
%define PIC1_IRQ_START          0x20

; ===========================================
; --- APIC Definitions ---
; ===========================================
; --- Local APIC Registers ---

%define LAPIC_REG_ID            0x020
%define LAPIC_REG_VERSION       0x030
%define LAPIC_REG_TPR           0x080
%define LAPIC_REG_APR           0x090
%define LAPIC_REG_PPR           0x0A0
%define LAPIC_REG_EOI           0x0B0
%define LAPIC_REG_LDR           0x0D0
%define LAPIC_REG_DFR           0x0E0  ; <<< WAS MISSING
%define LAPIC_REG_SPURIOUS      0x0F0
%define LAPIC_REG_ESR           0x280
%define LAPIC_REG_LVT_CMCI      0x2F0
%define LAPIC_REG_ICR_LOW       0x300
%define LAPIC_REG_ICR_HIGH      0x310
%define LAPIC_REG_LVT_TIMER     0x320
%define LAPIC_REG_LVT_THERMAL   0x330
%define LAPIC_REG_LVT_PERFMON   0x340
%define LAPIC_REG_LVT_LINT0     0x350
%define LAPIC_REG_LVT_LINT1     0x360
%define LAPIC_REG_LVT_ERROR     0x370
%define LAPIC_REG_TIMER_INITCNT 0x380
%define LAPIC_REG_TIMER_CURCNT  0x390
%define LAPIC_REG_TIMER_DIV     0x3E0

; LAPIC Spurious Vector Register Bits
%define LAPIC_SPURIOUS_VECTOR   0xFF
%define LAPIC_SPURIOUS_ENABLE   (1 << 8)
%define LAPIC_SPURIOUS_FOCUS    (1 << 9)

; LVT Register Bits
%define LVT_MASKED              (1 << 16)
%define LVT_DELIVERY_FIXED      (0 << 8)
%define LVT_DELIVERY_NMI        (4 << 8)
%define LVT_DELIVERY_EXTINT     (7 << 8)

; IA32_APIC_BASE MSR
%define MSR_APIC_BASE           0x1B
%define MSR_APIC_BASE_ENABLE    (1 << 11) ; <<< WAS MISSING

; --- I/O APIC Registers ---
%define IOAPIC_REG_ID           0x00
%define IOAPIC_REG_VER          0x01  ; <<< WAS MISSING
%define IOAPIC_REG_ARB          0x02
%define IOAPIC_REG_REDTBL_BASE  0x10

%define IOAPIC_INDEX_REG        0x00
%define IOAPIC_DATA_REG         0x10
%define IOAPIC_EOI_REG          0x40

; I/O APIC Redirection Table Entry (RTE - 64 bits)
%define RTE_VECTOR              0xFF
%define RTE_DELIVERY_MODE_FIXED (0 << 8)
%define RTE_DELIVERY_MODE_LP    (1 << 8)
%define RTE_DELIVERY_MODE_SMI   (2 << 8)
%define RTE_DELIVERY_MODE_NMI   (4 << 8)
%define RTE_DELIVERY_MODE_INIT  (5 << 8)
%define RTE_DELIVERY_MODE_EXTINT (7 << 8)
%define RTE_DEST_MODE_PHYSICAL  (0 << 11)
%define RTE_DEST_MODE_LOGICAL   (1 << 11)
%define RTE_DELIVERY_STATUS     (1 << 12)
%define RTE_INT_POLARITY_HIGH   (0 << 13)
%define RTE_INT_POLARITY_LOW    (1 << 13)
%define RTE_REMOTE_IRR          (1 << 14)
%define RTE_TRIGGER_MODE_EDGE   (0 << 15)
%define RTE_TRIGGER_MODE_LEVEL  (1 << 15)
%define RTE_MASKED              (1 << 16)
%define RTE_DESTINATION_SHIFT   56





; ===========================================
; --- END OF DEFINITIONS ---
; ===========================================

%endif ; BOOT_DEFS_TEMP_INC
