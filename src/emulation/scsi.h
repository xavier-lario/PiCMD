// Pi-CMD - A Commodore CMD-HD hard drive emulator
//
// SCSI disk (target) emulation.
// Ported from VICE's scsi.c/scsi.h written by Roberto Muscedere, adapted to
// FatFS backed image files with a write-back sector cache.
//
// This file is part of Pi1541.
//
// Pi1541 is free software : you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Pi1541 is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Pi1541. If not, see <http://www.gnu.org/licenses/>.

#ifndef SCSI_H
#define SCSI_H

#include "types.h"
#include "ff.h"

struct scsi_context_s;

// Write-path forensics: the payload ring and the bit-rot scan, both of which
// live inside WriteSector and therefore run on the emulation thread, once per
// 512 byte sector the computer writes.
//
// Off by default because they are not free, contrary to what the comment above
// BitRotWrites used to claim. A 512 byte memcpy plus a 512 iteration compare
// land on ONE iteration of a loop whose budget is 1.2us. PICMD-LST18.LOG, from
// a GEOS installation, counts 745 + 1656 + 113 = 2514 overruns of three, four
// and five microseconds, against 2515 sector writes in PICMD-WR00.LOG: one
// apiece, no exceptions. The drive's deadline for the first symbol of a
// transmitted byte is ten CPU cycles, 5us, and that symbol carries bits 0 and
// 1 - the bits this scan exists to watch being set. So the instrumentation is
// large enough to cause the fault it was written to catch, which is the same
// trap D4 of the timing handoff already recorded once.
//
// Set to 1 for a run that needs the payload log or the live rot counter, and
// read the resulting numbers knowing they were bought at that price.
#define PICMD_WRITE_FORENSICS 0

// The payload ring rides on the address log: a captured payload is paired with
// its address entry by accessLogCount, so there is nothing to pair it with when
// the address log is not there. Said here rather than left to the linker, which
// would report it as four unrelated missing symbols.
#if PICMD_WRITE_FORENSICS && !PICMD_ACCESS_FORENSICS
#error "PICMD_WRITE_FORENSICS needs PICMD_ACCESS_FORENSICS"
#endif

// A SCSI hard disk image backed by a FatFS file.
//
// Accesses go through a write-back cache of CACHE_CHUNK_SIZE chunks so that
// most of them do not touch the SD card. That matters more than throughput:
// an SD access blocks the emulated CPU inside a single instruction, and a
// frozen CPU cannot answer ATN, so the computer decides the drive is not
// there. Writes used to go straight through, which meant every 512 byte
// sector stalled the drive - and HD-TOOLS writes the system area one sector
// per SCSI command, polling for status in between.
//
// Write-back means the computer is told a write succeeded while the only copy
// of it is still in RAM, so everything below is built around never losing that
// copy: a chunk that fails to write keeps its dirty bits for the next attempt,
// a slot that cannot be written is not reused, and a failure that has already
// been acknowledged to the host comes back as a drive error rather than as
// silence (see HasWriteError).
class ScsiImage
{
public:
	static const u32 SECTOR_SIZE = 512;
	static const u32 CACHE_CHUNK_SHIFT = 12;			// 4K chunks (8 sectors)
	static const u32 CACHE_CHUNK_SIZE = 1 << CACHE_CHUNK_SHIFT;
	static const u32 SECTORS_PER_CHUNK = CACHE_CHUNK_SIZE / SECTOR_SIZE;

	// Everything, not just the three that used to be here: the images live in a
	// global today so .bss covers the rest, but an ScsiImage anywhere else
	// would start with a garbage imageId and match other images' cache slots.
	ScsiImage()
		: attached(false), sizeInSectors(0), readOnly(false), needSync(false),
		  writeErrorPending(false), imageId(0)
	{
		name[0] = 0;
	}

	bool Attach(const char* filename, bool readOnly);
	void Detach();
	bool IsAttached() const { return attached; }
	u32 SizeInSectors() const { return sizeInSectors; }
	bool IsReadOnly() const { return readOnly; }
	const char* GetFileName() const { return name; }

	// Read/write one 512 byte sector. Returns 0 on success.
	int ReadSector(u32 lba, u8* buffer);
	// Read one sector without disturbing (or filling) the cache. The whole
	// disk scans step 128 sectors at a time, so a 4K chunk fetch would throw
	// away seven eighths of what it reads and evict something useful.
	//
	// It will still answer out of a slot that happens to hold the sector,
	// which is free and cannot be stale - that is what separates it from
	// ReadSectorPhysical below.
	int ReadSectorUncached(u32 lba, u8* buffer);
	int WriteSector(u32 lba, const u8* buffer);

	// Read straight off the card, ignoring the cache entirely - not even as a
	// shortcut, which is what separates this from ReadSectorUncached.
	//
	// Only VERIFY wants this. Verifying through the cache verifies the cache:
	// a sector that was written a moment ago, or read earlier and still held,
	// comes back fine from a card that has since been removed or has stopped
	// answering. Flush before calling it, or what is on the card is not what
	// the computer believes it wrote.
	int ReadSectorPhysical(u32 lba, u8* buffer);

	// Longest single SD access so far, in microseconds. The bus timeout that
	// shows up as ?DEVICE NOT PRESENT is a few milliseconds, so this is the
	// number that says whether the cache is doing its job.
	static u32 WorstStallMicros() { return worstStallMicros; }
	static void ResetWorstStall() { worstStallMicros = 0; }

	// Bumped on every access, so the caller can tell whether the disk has been
	// left alone.
	static u32 AccessCounter() { return accessCounter; }

	// How many times a slot had to be taken from a chunk that still had
	// unwritten sectors in it. Every one of those is a synchronous write to
	// the card from inside ReadSector/WriteSector - the exact stall the cache
	// is supposed to prevent. If this stays at zero the cache is coping; if it
	// climbs during a transfer, that is where the drive is going deaf.
	static u32 DirtyEvictions() { return dirtyEvictions; }
	static void ResetDirtyEvictions() { dirtyEvictions = 0; }

	// Read instrumentation, for telling a cache coherency fault from a plain
	// slow card. Only ReadSector is counted; the whole disk scan deliberately
	// goes round the cache and would swamp the numbers.
	//
	// cacheReadHits   - answered from a slot without touching the card.
	// cacheReadMisses - had to go to the file.
	// dirtyChunkReads - landed on a chunk still holding unflushed writes.
	// dirtyChunkPartialFills - the subset of those that also had to pull a
	//   sector off the card into that same chunk. This is the only path in the
	//   cache that mixes what the card holds with writes that have not reached
	//   it, so if a stale block is ever handed back this is where it comes
	//   from. Zero here rules the cache out the way dirtyEvictions did.
	static u32 CacheReadHits() { return cacheReadHits; }
	static u32 CacheReadMisses() { return cacheReadMisses; }
	static u32 DirtyChunkReads() { return dirtyChunkReads; }
	static u32 DirtyChunkPartialFills() { return dirtyChunkPartialFills; }

	// Sectors the card refused and that were dropped when the image detached.
	// Anything other than zero here is user data that did not survive.
	static u32 UnflushedSectors() { return unflushedSectors; }

	// Bytes that gain bits 0 and 1 and nothing else between one write of a
	// sector and the next. That is the signature of the corruption eating the
	// BAM and directory sectors: never 0x01 alone, never 0x02 alone, always
	// both, and only ever in sectors the drive reads, modifies and writes back
	// itself. Caught here rather than by comparing image files afterwards, so
	// the first event names its own LBA while the machine is still running.
	//
	// Only checked when the sector's old contents are already sitting in the
	// cache, which is exactly the read-modify-write case - so this costs no
	// card access and cannot add a stall. Three bytes is the floor: real
	// events run to eleven or more, while a single byte flipping to a low bit
	// is ordinary data (a block count, a link) and would be noise. The scan
	// itself is only compiled in when PICMD_WRITE_FORENSICS is on, for the
	// reason given at the top of this file; with it off these read zero.
	static u32 BitRotWrites() { return bitRotWrites; }
	static u32 BitRotBytes() { return bitRotBytes; }
	static u32 BitRotFirstLba() { return bitRotFirstLba; }
	static u8 BitRotMask() { return bitRotMask; }

	// Zero every counter above, plus the stall watermark and the access log.
	// Called on mount so the figures describe what the computer went on to do
	// rather than the whole disk scan that precedes it - the scan alone is
	// imagesize/128 reads, which would fill the log before the computer has
	// asked for anything.
	static void ResetCounters();

	// Every sector access in order, so the pattern around a failure can be
	// read back afterwards. The contents are known good by now; what is not
	// known is where they were going. Held in RAM and written out on eject,
	// because going to the card while the bus is live is what breaks the
	// drive.
#if PICMD_ACCESS_FORENSICS
	static u32 AccessLogCount() { return accessLogCount; }
#endif

	// Highest LBA the computer has ever written, which is what says whether a
	// 100MB image is being used as the front 4MB of one. Maintained by
	// WriteSector itself and not by LogAccess: LogAccess compiles to nothing
	// unless PICMD_ACCESS_FORENSICS is on, and it is off in every build anyone
	// runs, so a figure kept there would read zero on the status line for ever
	// while looking like it worked. One compare and a store on a path that has
	// already committed to a 512 byte memcpy is affordable.
	static u32 MaxLbaWritten() { return maxLbaWritten; }

#if PICMD_ACCESS_FORENSICS
	static bool DumpAccessLog(const char* prefix);
#endif

	// The addresses turned out not to be enough: the data lands in the right
	// sectors and the file still comes out half its length, so what matters
	// now is the bytes. Keeps the whole 512 byte payload of the last writes.
#if PICMD_WRITE_FORENSICS
	static bool DumpWriteLog(const char* prefix);
#endif

	// Chunks written per visit to FlushIdle. Not "one card write": FlushChunk
	// writes each contiguous run of dirty sectors separately, so a 4K chunk
	// with an alternating dirtyMask costs up to four f_write calls. One chunk
	// is still the smallest unit the flush can be built out of, and it is what
	// an eviction already costs - FindSlot flushes a single dirty victim
	// synchronously - so this bounds the idle drain to a freeze the drive
	// already survives elsewhere. The caller comes straight back on the next
	// slow tick while the bus stays quiet, so a backlog drains promptly but in
	// slices the drive can answer ATN between.
	static const u32 IDLE_FLUSH_CHUNKS = 1;

	// Why the flush stopped. The three used to be one bool, which meant the
	// caller could not tell "there is more, come straight back" from "the card
	// said no" - and retrying a failed write every 256us instead of every half
	// second is a real-time regression for nothing, since FatFS latches the
	// error and the retry never reaches the card anyway.
	enum FlushResult
	{
		FLUSH_DONE,		// nothing left owed
		FLUSH_MORE,		// hit the per-visit bound; come back next slow tick
		FLUSH_STOPPED	// the bus wants us, or a write failed; back off
	};

	// Flush every attached image, a slice at a time. This is the one the
	// emulation loop's idle check calls, and the only flush that is safe to
	// call while the machine is up: it goes to the card, which freezes the
	// emulated CPU, and a frozen CPU cannot answer ATN.
	//
	// abort is checked before each chunk - and before the per-visit bound, so
	// that a bound of one does not turn it into a callback that can never
	// fire. Whatever is left stays dirty and goes out at the next quiet
	// moment, or at detach.
	static FlushResult FlushIdle(bool (*abort)() = 0);

	// How many chunks are written but not yet on the card. This is what would
	// be lost by pulling the power, so it is worth being able to see it.
	static u32 DirtyChunkCount();

	// Flush every attached image to the end. Returns the number of images that
	// could not be written. For detach and eject, where finishing matters more
	// than latency and there is no emulation left to starve; the idle check
	// wants FlushIdle instead, because this one is unbounded and a full cache
	// takes it far past anything the bus will wait through.
	static int FlushAll();

	// As FlushAll, but gives up after maxMicros of real time (0 = no limit).
	// Reset uses this: that path is reached from the IEC RESET line with the
	// host already running, so an unbounded flush would take the drive off the
	// bus for as long as the card needs - minutes, with a large dirty cache.
	//
	// The budget is time rather than chunks because a chunk is up to four
	// separate writes when its dirty sectors are not contiguous, so a chunk
	// count bounds the work only to within a factor of four. One deadline
	// covers the whole call, so it bounds the pass however many images are
	// mounted rather than bounding each of them separately.
	//
	// It is a SOFT budget. The deadline is tested before each write, and a
	// write in progress cannot be abandoned, so a call can overrun by one
	// access - just under 5ms at the worst rate measured here. A pass that
	// empties the cache also ends with f_sync, which is another. Treat
	// maxMicros as "stop starting new work after this", not as a ceiling.
	static int FlushSome(u32 maxMicros);

	// 50ms, which is about ten card accesses: the worst single access measured
	// on this hardware is 4,959us. It was five times that while an access was
	// believed to cost 35ms, a figure that turned out to be mostly the
	// millisecond granularity of the poll in CEMMCDevice::TimeoutWait rather
	// than anything the card was doing. Whatever does not go out inside the
	// budget stays dirty and is drained by FlushIdle at the next quiet moment,
	// so budgeting low costs a delay, not data.
	static const u32 RESET_FLUSH_MICROS = 50000;

	// Flush this image's dirty chunks and FatFS metadata. Does nothing unless
	// forced - going to the card at an arbitrary moment is what broke the bus.
	// Detach and reset force it; everything else waits for an idle window.
	//
	// deadline is an absolute ARM_SYSTIMER_CLO value, 0 for no limit; one
	// deadline shared by several images bounds the whole flush rather than
	// each image separately. Returns 0 if everything went out, 1 if dirty
	// chunks remain, negative on a write failure - which is a different thing
	// from 1, because the data is still there either way but only one of them
	// is going to fix itself.
	int Sync(bool force = false, u32 deadline = 0);

	// Writes are acknowledged to the host as soon as they reach the cache, so
	// by the time a flush fails the computer has long since been told the
	// write succeeded. There is no way to un-acknowledge it, so instead the
	// data stays dirty for a later retry and this latches - the next command
	// for the image reports CHECK CONDITION / MEDIUM ERROR so the failure
	// surfaces as a drive error rather than as silent corruption.
	bool HasWriteError() const { return writeErrorPending; }
	bool TakeWriteError() { bool e = writeErrorPending; writeErrorPending = false; return e; }

	// Total flush failures since boot, for the display and for bug reports.
	static u32 WriteErrorCount() { return writeErrors; }

	// Read the front of the image into the cache and pin it, so that region
	// never costs an SD access again. Call it at mount and nowhere else: it
	// goes to the card for as long as it takes, which is only safe while the
	// bus is quiet. progressSectors/totalSectors drive the on screen bar and
	// may be null. Returns the number of chunks actually loaded.
	u32 PreloadCache(u32 maxBytes, volatile u32* progressSectors, volatile u32* totalSectors);
	static u32 PinnedKB() { return (pinnedChunks * CACHE_CHUNK_SIZE) >> 10; }

	// The (global) cache used by all images.
	static void InitCache(u32 sizeInBytes);
	// How much cache was actually obtained. Zero means every access goes
	// straight to the card, so it is worth showing at boot.
	static u32 CacheSizeKB() { return (numCacheSlots * CACHE_CHUNK_SIZE) >> 10; }

private:
	FIL file;
	bool attached;
	u32 sizeInSectors;
	bool readOnly;
	bool needSync;
	bool writeErrorPending;
	static u32 worstStallMicros;
	static u32 accessCounter;
	static u32 writeErrors;
	static u32 dirtyEvictions;
	static u32 cacheReadHits;
	static u32 cacheReadMisses;
	static u32 dirtyChunkReads;
	static u32 dirtyChunkPartialFills;
	static u32 unflushedSectors;
	static u32 bitRotWrites;
	static u32 bitRotBytes;
	static u32 bitRotFirstLba;
	static u8 bitRotMask;

	// 16K entries at 8 bytes is 128KB of BSS, and covers a GEOS install with
	// room to spare. If it ever wraps, accessLogCount says so and the dump
	// prints the surviving tail in order.
	static const u32 ACCESS_LOG_ENTRIES = 16384;
	struct AccessLogEntry
	{
		u32 lba;
		u8 op;			// 'R' cached read, 'W' write, 'U' uncached read
		u8 image;
		u8 result;		// 0 = accepted, 1 = refused (out of range, detached)
		u8 pad;
	};
#if PICMD_ACCESS_FORENSICS
	static AccessLogEntry accessLog[ACCESS_LOG_ENTRIES];
	static u32 accessLogCount;		// total accesses, not entries held
#endif
	static u32 maxLbaWritten;
#if PICMD_ACCESS_FORENSICS
	void LogAccess(u32 lba, u8 op, u8 result);
#else
	// Compiled to nothing rather than removed from the call sites: the calls
	// mark the three places an access enters the image, which is worth keeping
	// visible even when nobody is recording them.
	inline void LogAccess(u32, u8, u8) {}
#endif

	// 256 payloads of 512 bytes is 128KB, and covers every write a whole file
	// copy makes with room over.
	static const u32 WRITE_LOG_ENTRIES = 256;
#if PICMD_WRITE_FORENSICS
	static u8 writeLogData[WRITE_LOG_ENTRIES][SECTOR_SIZE];
	static u32 writeLogLba[WRITE_LOG_ENTRIES];
	static u32 writeLogSeq[WRITE_LOG_ENTRIES];	// index into the access log
	static u32 writeLogCount;
#endif

	char name[256];

	struct CacheSlot
	{
		u8* data;
		u32 chunkIndex;
		u8 image;		// which image (index into a small registry)
		u8 valid;		// slot is allocated to this chunk
		// Per sector, because a write must never have to read first. Reading
		// the chunk in just to modify one sector of it meant an SD access on
		// the first write to every chunk, and an SD access freezes the drive
		// for as long as the card feels like - 38ms, measured. The computer
		// gives up on us long before that.
		u8 validMask;	// sectors holding real data
		u8 dirtyMask;	// sectors not yet on the card
	};

	// Push one dirty chunk out to the card, in as few writes as the runs of
	// set bits allow. Used when a slot is reused for something else, and by
	// FlushAllDirty.
	//
	// deadline is the absolute ARM_SYSTIMER_CLO value described on FlushSome,
	// 0 for none, and is checked between runs rather than once per chunk - a
	// fragmented chunk is up to four card accesses, and a budget that only
	// looked at whole chunks would overrun by three of them. Eviction passes
	// none: it has to get the whole chunk out before the slot can be reused,
	// and there is nowhere else for the data to go.
	//
	// Returns 0 when nothing failed, which is not the same as "the chunk is
	// now clean" - the deadline can stop it part way through. Only the bits
	// that actually reached the card are cleared, so what is left simply stays
	// dirty; decide whether there is more to do from dirtyMask, not from the
	// return value. A chunk that fails likewise keeps its dirty bits, so the
	// next flush tries again rather than dropping the data on the floor.
	int FlushChunk(CacheSlot& slot, u32 deadline = 0);

	u32 DirtySectorCount();

	// Whether this image still has chunks waiting to be written. Read only -
	// callers use it to decide whether a flush finished, so it must not be
	// tempted into writing anything itself.
	bool HasDirtyChunks() const;

	// Push this image's dirty chunks out to the card under both bounds at
	// once. Neither is a nicety: the emulated CPU is frozen for every card
	// write in this loop, and unbounded that came to 48 milliseconds in one go
	// on a GEOS installation - measured, the worst overrun in PICMD-LST37.LOG.
	//
	// abort is asked before each chunk, and before the chunk bound, so that a
	// bound of one does not turn it into a callback that can never fire.
	// maxChunks stops after that many chunks; 0 means no chunk bound. deadline
	// is the absolute ARM_SYSTIMER_CLO value described on FlushSome; 0 means
	// no time bound.
	//
	// The two bounds answer different callers and are not meant to overlap.
	// The idle drain wants the smallest slice it can come back from and passes
	// (abort, IDLE_FLUSH_CHUNKS, 0); the reset flush wants as much as fits in
	// a fixed piece of wall clock and passes (0, 0, deadline). Whichever bound
	// is reached first ends the pass, and either way what is left stays dirty
	// for the next visit.
	//
	// FLUSH_MORE means a bound stopped it, FLUSH_STOPPED that the abort fired
	// or a write failed. Those two are the same instruction to the caller -
	// back off - and a caller that passes no abort can only be seeing the
	// second, which is what lets Sync report a write failure without needing a
	// second return value.
	FlushResult FlushAllDirty(bool (*abort)(), u32 maxChunks, u32 deadline);

	// Write a slot back so it can be reused, through whoever owns it. False
	// means the data could not be written and the slot must be left where it
	// is - the cache is the only copy, so evicting it anyway is data loss.
	static bool EvictSlot(CacheSlot* victim);
	static void NoteStall(u32 startMicros);

	// A chunk being evicted can belong to a different disk, so we need to get
	// back from the id stored in the slot to the image that owns the file.
	static ScsiImage* attachedImages[64];
	static u32 numAttachedImages;
	static ScsiImage* ImageById(u8 id);

	static u8* cachePool;
	static CacheSlot* cacheSlots;
	static u32 numCacheSlots;

	// Slots [0, pinnedChunks) belong to image pinnedImage, chunk N in slot N.
	// The hash covers only what is left, so nothing can collide with them.
	static u32 pinnedChunks;
	static u8 pinnedImage;

	// Lowest id not currently taken by an attached image, handed out in
	// Attach. A counter that only ever went up was simpler and wrong: it wraps
	// after enough mounts, and the moment it does, a fresh image matches the
	// cache slots of an older one and reads back its sectors.
	u8 imageId;

	int FillChunk(u32 chunkIndex, CacheSlot& slot);
	CacheSlot* FindSlot(u32 chunkIndex, bool allocate);
};

// SCSI bus states as presented by U13 to the CMD HD (see cmdhd glue).
#define SCSI_STATE_DATAOUT    0x00
#define SCSI_STATE_DATAIN     0x01
#define SCSI_STATE_COMMAND    0x02
#define SCSI_STATE_STATUS     0x03
#define SCSI_STATE_MESSAGEOUT 0x04
#define SCSI_STATE_MESSAGEIN  0x05
#define SCSI_STATE_BUSFREE    0x10
#define SCSI_STATE_SELECTION  0x20

#define SCSI_STATUS_GOOD                     0x00
#define SCSI_STATUS_CHECKCONDITION           0x01

#define SCSI_SENSEKEY_NOSENSE        0x00
#define SCSI_SENSEKEY_NOTREADY       0x02
#define SCSI_SENSEKEY_MEDIUMERROR    0x03
#define SCSI_SENSEKEY_ILLEGALREQUEST 0x05

#define SCSI_SASC_WRITEFAULT                    0x03
#define SCSI_SASC_UNRECOVEREDREADERROR          0x11
#define SCSI_SASC_MEDIUMNOTPRESENT              0x3A
#define SCSI_SASC_LOGICALBLOCKADDRESSOUTOFRANGE 0x21
#define SCSI_SASC_INVALIDFIELDINCDB             0x24
#define SCSI_SASC_INVALIDFIELDINPARAMETERLIST   0x26

#define SCSI_LOG_NODISK0 0x01

#define SCSI_COMMAND_TEST_UNIT_READY       0x00
#define SCSI_COMMAND_REZERO_UNIT           0x01
#define SCSI_COMMAND_REQUEST_SENSE         0x03
#define SCSI_COMMAND_FORMAT_UNIT           0x04
#define SCSI_COMMAND_REASSIGN_BLOCKS       0x07
#define SCSI_COMMAND_READ_6                0x08
#define SCSI_COMMAND_WRITE_6               0x0a
#define SCSI_COMMAND_INQUIRY               0x12
#define SCSI_COMMAND_MODE_SENSE            0x1a
#define SCSI_COMMAND_START_STOP            0x1b
#define SCSI_COMMAND_SEND_DIAGNOSTIC       0x1d
#define SCSI_COMMAND_READ_CAPACITY         0x25
#define SCSI_COMMAND_READ_10               0x28
#define SCSI_COMMAND_WRITE_10              0x2a
#define SCSI_COMMAND_VERIFY                0x2f
#define SCSI_COMMAND_MODE_SENSE_10         0x5a
#define SCSI_COMMAND_WRITE_VERIFY          0x2e

#define SCSI_MAX_DISKS 56

typedef struct scsi_context_s
{
	u8 state;
	u8 target;
	u8 databus;
	u8 ack;
	u8 req;
	u8 bsyi;
	u8 bsyo;
	u8 sel;
	u8 rst;
	u8 atn;
	u8 cd;
	u8 io;
	u8 msg;
	u32 seq;
	u32 cmd_size;
	u32 address;
	u32 blocks;
	u32 data_max;
	u8 link;
	u8 status;
	u8 lun;
	u8 command;
	u8 sensekey;
	u8 asc;
	u8 cmd_buf[256];
	u8 data_buf[512];
	u8 msg_after_status;
	u32 max_imagesize;		/* in 512 byte sectors; 0 = use image length */
	u32 limit_imagesize;	/* in 512 byte sectors */
	u32 log;
	ScsiImage* file[SCSI_MAX_DISKS];
	void* p;
	// Returns 0 on success. FORMAT UNIT used to report GOOD whatever the
	// handler did, so a format onto read-only media or a failing card looked
	// like it had worked.
	s32 (*user_format)(struct scsi_context_s*);
	void (*user_read)(struct scsi_context_s*);
	void (*user_write)(struct scsi_context_s*);
} scsi_context_t;

s32 scsi_image_read(scsi_context_t* context);
s32 scsi_image_read_uncached(scsi_context_t* context);
s32 scsi_image_write(scsi_context_t* context);
u8 scsi_get_bus(scsi_context_t* context);
int scsi_set_bus(scsi_context_t* context, u8 value);
void scsi_process_noack(scsi_context_t* context);
void scsi_process_ack(scsi_context_t* context);
void scsi_reset(scsi_context_t* context);

#endif
