// Created by Apehaenger on 02/02/23.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based on it without getting my consent first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//
#ifndef _DEBUG_H_
#define _DEBUG_H_

// Define to stream debugging messages via USB
//#define USB_DEBUG

// Only define DEBUG_SERIAL if USB_DEBUG is actually enabled.
// This enforces compile errors if it's used incorrectly.
#ifdef USB_DEBUG
#define DEBUG_SERIAL Serial
#define DfMiniMp3Debug DEBUG_SERIAL
// Some blody simple debug wrapper which superfluous '#ifdef USB_DEBUG' ...
#define DEBUG_PRINT(s)            \
    {                             \
        DEBUG_SERIAL.print(F(s)); \
    }
#define DEBUG_PRINTLN(s)            \
    {                               \
        DEBUG_SERIAL.println(F(s)); \
    }
#define DEBUG_PRINT_VAL(v, f)     \
    do                            \
    {                             \
        DEBUG_SERIAL.print(v, f); \
    } while (0);
#define DEBUG_PRINTLN_VAL(v, f)     \
    do                              \
    {                               \
        DEBUG_SERIAL.println(v, f); \
    } while (0);
#else
#define DEBUG_PRINT(s)
#define DEBUG_PRINTLN(s)
#define DEBUG_PRINT_VAL(v, f)
#define DEBUG_PRINTLN_VAL(v, f)
#endif

#endif // _DEBUG_H_
