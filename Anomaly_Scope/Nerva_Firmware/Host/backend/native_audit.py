"""Bounded application-to-native-file accounting; no hardware loss claim."""
import hashlib
import struct


def sample_position(offset,bps,spts):
    if bps<=0:raise ValueError('Missing bytes/sample')
    if not spts:return offset//bps,offset%bps
    groups,tail=divmod(offset,bps*spts+8)
    return groups*spts+min(spts,tail//bps),(tail%bps if tail<bps*spts else 0)


class NativeAudit:
    def __init__(self,item_size,bps,spts,delivered=None,filex_bytes=None):
        if not 4<item_size<=131072 or bps<=0 or spts<0:
            raise ValueError('Invalid bounded native layout')
        self.item_size,self.bps,self.spts=item_size,bps,spts
        self.delivered,self.filex_bytes=delivered,filex_bytes
        self.pending=bytearray();self.digest=hashlib.sha256()
        self.bytes=self.payload=self.blocks=self.error_count=0
        self.errors=[];self.finished=False

    def error(self,message):
        self.error_count+=1
        if len(self.errors)<32:self.errors.append(message)

    def block(self):
        if len(self.pending)<4:
            self.error('Incomplete native counter header')
        else:
            counter=struct.unpack_from('<I',self.pending)[0]
            if counter!=(self.payload&0xffffffff):
                self.error(dict(block=self.blocks,expected_counter=self.payload&0xffffffff,actual_counter=counter))
            self.payload+=len(self.pending)-4;self.blocks+=1
        self.pending.clear()

    def feed(self,data):
        if self.finished:raise ValueError('Native audit already finalized')
        self.digest.update(data);self.bytes+=len(data)
        view=memoryview(data);pos=0
        while pos<len(view):
            n=min(self.item_size-len(self.pending),len(view)-pos)
            self.pending.extend(view[pos:pos+n]);pos+=n
            if len(self.pending)==self.item_size:self.block()

    def finish(self):
        if self.finished:raise ValueError('Native audit already finalized')
        self.finished=True
        if self.pending:self.block()
        samples,remainder=sample_position(self.payload,self.bps,self.spts)
        if remainder:self.error('File ends within a sample')
        if self.spts and self.payload%(self.bps*self.spts+8)>=self.bps*self.spts:
            self.error('File ends with a missing or incomplete timestamp after a complete sample group')
        if self.delivered is not None and samples!=self.delivered:
            self.error(f'Delivered/application samples {self.delivered} differ from SD samples {samples}')
        if self.filex_bytes is not None and self.bytes!=self.filex_bytes:
            self.error(f'FileX accepted bytes {self.filex_bytes} differ from on-card file bytes {self.bytes}')
        return dict(file_bytes=self.bytes,payload_bytes=self.payload,blocks=self.blocks,
            samples_represented=samples,application_delivered_samples=self.delivered,
            missing_from_sd=None if self.delivered is None else self.delivered-samples,
            sha256=self.digest.hexdigest(),accounting_result='FAIL' if self.error_count else 'PASS',
            errors=self.errors,error_count=self.error_count,hardware_zero_loss_pass=False)
