#![feature(asm)]
#![feature(naked_functions)]
use std::ptr;

const DEFAULT_STACK_SIZE: usize = 1024 * 1024 * 2;
const MAX_THREADS: usize = 4;
static mut RUNTIME: usize = 0;

pub struct Runtime {
    threads: Vec<Thread>,
    current: usize,
}

#[derive(PartialEq, Eq, Debug)]
enum State {
    Available,
    Running,
    Ready,
}

struct Thread {
    id: usize,
    stack: Vec<u8>,
    ctx: ThreadContext,
    state: State,
}

#[cfg(not(target_os = "windows"))]
#[derive(Debug, Default)]
#[repr(C)]
struct ThreadContext {
    rsp: u64,
    r15: u64,
    r14: u64,
    r13: u64,
    r12: u64,
    rbx: u64,
    rbp: u64,
}

impl Thread {
    fn new(id: usize) -> Self {
        Thread {
            id,
            stack: vec![0_u8; DEFAULT_STACK_SIZE],
            ctx: ThreadContext::default(),
            state: State::Available,
        }
    }
}

impl Runtime {
    pub fn new() -> Self {
        let base_thread = Thread {
            id: 0,
            stack: vec![0_u8; DEFAULT_STACK_SIZE],
            ctx: ThreadContext::default(),
            state: State::Running,
        };

        let mut threads = vec![base_thread];
        let mut available_threads: Vec<Thread> = (1..MAX_THREADS).map(|i| Thread::new(i)).collect();
        threads.append(&mut available_threads);

        Runtime {
            threads,
            current: 0,
        }
    }

    pub fn init(&self) {
        unsafe {
            let r_ptr: *const Runtime = self;
            RUNTIME = r_ptr as usize;
        }
    }

    pub fn run(&mut self) -> ! {
        while self.t_yield() {}
        std::process::exit(0);
    }

    fn t_return(&mut self) {
        if self.current != 0 {
            self.threads[self.current].state = State::Available;
            self.t_yield();
        }
    }
    fn t_yield(&mut self) -> bool {
        let mut pos = self.current;
        while self.threads[pos].state != State::Ready {
            pos += 1;
            if pos == self.threads.len() {
                pos = 0;
            }
            if pos == self.current {
                return false;
            }
        }

        if self.threads[self.current].state != State::Available {
            self.threads[self.current].state = State::Ready;
        }

        self.threads[pos].state = State::Running;
        let old_pos = self.current;
        self.current = pos;

        unsafe {
            switch(&mut self.threads[old_pos].ctx, &self.threads[pos].ctx);
        }

        true
    }

    #[cfg(not(target_os = "windows"))]
    pub fn spawn(&mut self, f: fn()) {
        let available = self
            .threads
            .iter_mut()
            .find(|t| t.state == State::Available)
            .expect("no available thread.");

        let size = available.stack.len();
        let s_ptr = available.stack.as_mut_ptr();
        unsafe {
            ptr::write(s_ptr.offset((size - 8) as isize) as *mut u64, guard as u64);
            ptr::write(s_ptr.offset((size - 16) as isize) as *mut u64, f as u64);
            available.ctx.rsp = s_ptr.offset((size - 16) as isize) as u64;
        }
        available.state = State::Ready;
    }
}

#[cfg_attr(any(target_os = "windows", target_os = "linux"), naked)]
fn guard() {
    unsafe {
        let rt_ptr = RUNTIME as *mut Runtime;
        let rt = &mut *rt_ptr;
        println!("THREAD {} FINISHED.", rt.threads[rt.current].id);
        rt.t_return();
    };
}

pub fn yield_thread() {
    unsafe {
        let rt_ptr = RUNTIME as *mut Runtime;
        (*rt_ptr).t_yield();
    };
}

#[cfg(not(target_os = "windows"))]
#[naked]
unsafe fn switch(old: *mut ThreadContext, new: *const ThreadContext) {
    asm!("
        mov     $0, %rdi
        mov     %rsp, 0x00(%rdi)
        mov     %r15, 0x08(%rdi)
        mov     %r14, 0x10(%rdi)
        mov     %r13, 0x18(%rdi)
        mov     %r12, 0x20(%rdi)
        mov     %rbx, 0x28(%rdi)
        mov     %rbp, 0x30(%rdi)

        mov     $1, %rsi
        mov     0x00(%rsi), %rsp
        mov     0x08(%rsi), %r15
        mov     0x10(%rsi), %r14
        mov     0x18(%rsi), %r13
        mov     0x20(%rsi), %r12
        mov     0x28(%rsi), %rbx
        mov     0x30(%rsi), %rbp

        ret
        "
    :
    :"{rdi}"(old), "{rsi}"(new)
    :
    : "volatile", "alignstack"
    );
}

fn main() {
    let mut runtime = Runtime::new();
    runtime.init();
    runtime.spawn(|| {
        println!("THREAD 1 STARTING");
        let id = 1;
        for i in 0..10 {
            println!("thread: {} counter: {}", id, i);
            yield_thread();
        }
    });
    runtime.spawn(|| {
        println!("THREAD 2 STARTING");
        let id = 2;
        for i in 0..15 {
            println!("thread: {} counter: {}", id, i);
            yield_thread();
        }
    });
    runtime.run();
}

// ===== WINDOWS SUPPORT =====
#[cfg(target_os = "windows")]
#[derive(Debug, Default)]
#[repr(C)]
struct ThreadContext {
    rsp: u64,
    r15: u64,
    r14: u64,
    r13: u64,
    r12: u64,
    rbx: u64,
    rbp: u64,
    xmm6: u64,
    xmm7: u64,
    xmm8: u64,
    xmm9: u64,
    xmm10: u64,
    xmm11: u64,
    xmm12: u64,
    xmm13: u64,
    xmm14: u64,
    xmm15: u64,
    stack_start: u64,
    stack_end: u64,
}

impl Runtime {
    #[cfg(target_os = "windows")]
    pub fn spawn(&mut self, f: fn()) {
        let available = self
            .threads
            .iter_mut()
            .find(|t| t.state == State::Available)
            .expect("no available thread.");

        let size = available.stack.len();
        let s_ptr = available.stack.as_mut_ptr();
        unsafe {
            ptr::write(s_ptr.offset((size - 8) as isize) as *mut u64, guard as u64);
            ptr::write(s_ptr.offset((size - 16) as isize) as *mut u64, f as u64);
            available.ctx.rsp = s_ptr.offset((size - 16) as isize) as u64;
            available.ctx.stack_start = s_ptr.offset(size as isize) as u64;
        }
        available.ctx.stack_end = s_ptr as *const u64 as u64;

        available.state = State::Ready;
    }
}

// reference: https://probablydance.com/2013/02/20/handmade-coroutines-for-windows/
// Contents of TIB on Windows: https://en.wikipedia.org/wiki/Win32_Thread_Information_Block
#[cfg(target_os = "windows")]
#[naked]
unsafe fn switch(old: *mut ThreadContext, new: *const ThreadContext) {
    asm!("
        mov     $0, %rdi
        mov     %rsp, 0x00(%rdi)
        mov     %r15, 0x08(%rdi)
        mov     %r14, 0x10(%rdi)
        mov     %r13, 0x18(%rdi)
        mov     %r12, 0x20(%rdi)
        mov     %rbx, 0x28(%rdi)
        mov     %rbp, 0x30(%rdi)
        mov     %xmm6, 0x38(%rdi)
        mov     %xmm7, 0x40(%rdi)
        mov     %xmm8, 0x48(%rdi)
        mov     %xmm9, 0x50(%rdi)
        mov     %xmm10, 0x58(%rdi)
        mov     %xmm11, 0x60(%rdi)
        mov     %xmm12, 0x68(%rdi)
        mov     %xmm13, 0x70(%rdi)
        mov     %xmm14, 0x78(%rdi)
        mov     %xmm15, 0x80(%rdi)
        mov     %gs:0x08, %rax     # windows support
        mov     %rax, 0x88(%rdi)   # windows support
        mov     %gs:0x10, %rax     # windows support
        mov     %rax, 0x90(%rdi)   # windows support

        mov     $1, %rsi
        mov     0x00(%rsi), %rsp
        mov     0x08(%rsi), %r15
        mov     0x10(%rsi), %r14
        mov     0x18(%rsi), %r13
        mov     0x20(%rsi), %r12
        mov     0x28(%rsi), %rbx
        mov     0x30(%rsi), %rbp
        mov     0x38(%rsi), %xmm6
        mov     0x40(%rsi), %xmm7
        mov     0x48(%rsi), %xmm8
        mov     0x50(%rsi), %xmm9
        mov     0x58(%rsi), %xmm10
        mov     0x60(%rsi), %xmm11
        mov     0x68(%rsi), %xmm12
        mov     0x70(%rsi), %xmm13
        mov     0x78(%rsi), %xmm14
        mov     0x80(%rsi), %xmm15
        mov     0x88(%rsi), %rax   # windows support
        mov     %rax, %gs:0x08     # windows support
        mov     0x90(%rsi), %rax   # windows support 
        mov     %rax, %gs:0x10     # windows support

        ret
        "
    :
    :"{rdi}"(old), "{rsi}"(new)
    :
    : "volatile", "alignstack"
    );
}
