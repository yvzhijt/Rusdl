//! A small arithmetic language for custom source waveforms.
//!
//! The user writes something like `3 - sin(t - pi/2)` and it is evaluated against the
//! simulation time in seconds, with angles in radians. That is deliberately not a scripting
//! language: there is no assignment, no state, no way to loop, and no way to reach anything
//! outside the arithmetic. A source's waveform is re-evaluated every timestep, so anything
//! with state would make the result depend on how the solver happened to iterate rather than
//! on the time.
//!
//! The grammar is the usual arithmetic one, with `^` for powers and a fixed function table:
//!
//! ```text
//! expression := term (('+' | '-') term)*
//! term       := unary (('*' | '/' | '%') unary)*
//! unary      := ('-' | '+') unary | power
//! power      := atom ('^' unary)?        // right associative, so 2^-1 works
//! atom       := number | 't' | constant | call | '(' expression ')'
//! call       := name '(' expression (',' expression)* ')'
//! ```
//!
//! Arithmetic follows IEEE 754, so `1/0` is an infinity rather than an error. That is a
//! deliberate choice: an expression that goes bad at one particular `t` is a modelling
//! mistake, and the honest response is to notice it when the value comes back non-finite
//! rather than to reject the whole expression up front, which would also reject `1/t` at `t=0`
//! where it is perfectly fine.

use std::fmt;

/// A parse failure, with the byte offset it happened at so a caller can point at it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExprError {
    /// Byte offset into the source text.
    pub offset: usize,
    /// What was wrong, in a form worth showing a user.
    pub message: String,
}

impl ExprError {
    fn at(offset: usize, message: impl Into<String>) -> Self {
        Self {
            offset,
            message: message.into(),
        }
    }
}

impl fmt::Display for ExprError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "第 {} 个字符处：{}", self.offset + 1, self.message)
    }
}

impl std::error::Error for ExprError {}

/// Binary operators, in the order the parser binds them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum BinOp {
    Add,
    Sub,
    Mul,
    Div,
    Rem,
    Pow,
}

/// A parsed expression.
///
/// Opaque on purpose: the tree is an implementation detail, so it can be reshaped without
/// breaking anyone, and callers get [Expr::parse] and [Expr::eval] and nothing else to
/// depend on.
#[derive(Clone, Debug, PartialEq)]
pub struct Expr {
    root: Node,
}

/// One node of the parsed tree.
#[derive(Clone, Debug, PartialEq)]
enum Node {
    Number(f64),
    /// The simulation time in seconds.
    Time,
    /// `pi`, `e` or `tau`.
    Constant(&'static str),
    Neg(Box<Node>),
    Binary {
        op: BinOp,
        lhs: Box<Node>,
        rhs: Box<Node>,
    },
    Call {
        name: &'static str,
        args: Vec<Node>,
    },
}

impl Expr {
    /// Parses `text`, or says where and why it could not.
    pub fn parse(text: &str) -> Result<Self, ExprError> {
        let tokens = tokenize(text)?;
        let mut parser = Parser {
            tokens: &tokens,
            pos: 0,
        };
        let root = parser.expression(0)?;
        if let Some((_, offset)) = parser.peek() {
            return Err(ExprError::at(*offset, "多余的符号"));
        }
        Ok(Self { root })
    }

    /// The value at simulation time `t`, in seconds.
    ///
    /// Follows IEEE 754, so a domain error such as `ln(-1)` or a division by zero comes back
    /// as a NaN or an infinity rather than as a panic. The caller is expected to check.
    pub fn eval(&self, t: f64) -> f64 {
        self.root.eval(t)
    }
}

impl Node {
    fn eval(&self, t: f64) -> f64 {
        match self {
            Self::Number(value) => *value,
            Self::Time => t,
            Self::Constant(name) => match *name {
                "pi" => std::f64::consts::PI,
                "tau" => std::f64::consts::TAU,
                _ => std::f64::consts::E,
            },
            Self::Neg(inner) => -inner.eval(t),
            Self::Binary { op, lhs, rhs } => {
                let (lhs, rhs) = (lhs.eval(t), rhs.eval(t));
                match op {
                    BinOp::Add => lhs + rhs,
                    BinOp::Sub => lhs - rhs,
                    BinOp::Mul => lhs * rhs,
                    BinOp::Div => lhs / rhs,
                    BinOp::Rem => lhs % rhs,
                    BinOp::Pow => lhs.powf(rhs),
                }
            }
            Self::Call { name, args } => {
                // Every function in the table is called with exactly the arity checked at
                // parse time, so indexing is in range and there is nothing to fall back on.
                let values: Vec<f64> = args.iter().map(|arg| arg.eval(t)).collect();
                call(name, &values)
            }
        }
    }
}

/// How many arguments each known function takes. The names are the ones a person who has
/// used a calculator is likely to try, and the arities are checked when parsing so a
/// mistyped call is reported where it was written.
fn arity(name: &str) -> Option<usize> {
    Some(match name {
        "sin" | "cos" | "tan" | "asin" | "acos" | "atan" | "sinh" | "cosh" | "tanh" | "exp"
        | "ln" | "log" | "log10" | "sqrt" | "cbrt" | "abs" | "sign" | "floor" | "ceil"
        | "round" | "trunc" | "sqr" => 1,
        "min" | "max" | "pow" | "atan2" | "hypot" | "mod" => 2,
        _ => return None,
    })
}

fn call(name: &'static str, args: &[f64]) -> f64 {
    match name {
        "sin" => args[0].sin(),
        "cos" => args[0].cos(),
        "tan" => args[0].tan(),
        "asin" => args[0].asin(),
        "acos" => args[0].acos(),
        "atan" => args[0].atan(),
        "sinh" => args[0].sinh(),
        "cosh" => args[0].cosh(),
        "tanh" => args[0].tanh(),
        "exp" => args[0].exp(),
        "ln" => args[0].ln(),
        // `log` as base 10 because that is what it means on a calculator; the natural
        // logarithm is `ln`, which is what a textbook usually writes.
        "log" | "log10" => args[0].log10(),
        "sqrt" => args[0].sqrt(),
        "cbrt" => args[0].cbrt(),
        "abs" => args[0].abs(),
        "sign" => {
            if args[0] == 0.0 {
                0.0
            } else {
                args[0].signum()
            }
        }
        "floor" => args[0].floor(),
        "ceil" => args[0].ceil(),
        "round" => args[0].round(),
        "trunc" => args[0].trunc(),
        // Squaring without a multiply, since it comes up constantly in waveforms.
        "sqr" => args[0] * args[0],
        "min" => args[0].min(args[1]),
        "max" => args[0].max(args[1]),
        "pow" => args[0].powf(args[1]),
        "atan2" => args[0].atan2(args[1]),
        "hypot" => args[0].hypot(args[1]),
        "mod" => args[0] % args[1],
        // Unreachable: `arity` rejects anything not in the list above, and `parse` checks it.
        _ => f64::NAN,
    }
}

/// A token and where it started.
type Spanned = (Token, usize);

#[derive(Clone, Debug, PartialEq)]
enum Token {
    Number(f64),
    Ident(String),
    Op(BinOp),
    LParen,
    RParen,
    Comma,
}

fn tokenize(text: &str) -> Result<Vec<Spanned>, ExprError> {
    let bytes = text.as_bytes();
    let mut tokens = Vec::new();
    let mut i = 0;
    while i < bytes.len() {
        let c = bytes[i] as char;
        match c {
            c if c.is_ascii_whitespace() => i += 1,
            c if c.is_ascii_digit() || c == '.' => {
                let start = i;
                while i < bytes.len() && ((bytes[i] as char).is_ascii_digit() || bytes[i] == b'.') {
                    i += 1;
                }
                // An exponent, but only when the `e` is followed by digits or a sign, so that
                // `2e` is reported as a bad name rather than as the number 2.
                if i < bytes.len() && (bytes[i] == b'e' || bytes[i] == b'E') {
                    let mut j = i + 1;
                    if j < bytes.len() && (bytes[j] == b'+' || bytes[j] == b'-') {
                        j += 1;
                    }
                    if j < bytes.len() && (bytes[j] as char).is_ascii_digit() {
                        while j < bytes.len() && (bytes[j] as char).is_ascii_digit() {
                            j += 1;
                        }
                        i = j;
                    }
                }
                let literal = &text[start..i];
                let value = literal
                    .parse::<f64>()
                    .map_err(|_| ExprError::at(start, format!("无法识别的数字：{literal}")))?;
                tokens.push((Token::Number(value), start));
            }
            c if c.is_ascii_alphabetic() || c == '_' => {
                let start = i;
                while i < bytes.len()
                    && ((bytes[i] as char).is_ascii_alphanumeric() || bytes[i] == b'_')
                {
                    i += 1;
                }
                tokens.push((Token::Ident(text[start..i].to_ascii_lowercase()), start));
            }
            '+' => {
                tokens.push((Token::Op(BinOp::Add), i));
                i += 1;
            }
            '-' => {
                tokens.push((Token::Op(BinOp::Sub), i));
                i += 1;
            }
            '*' => {
                tokens.push((Token::Op(BinOp::Mul), i));
                i += 1;
            }
            '/' => {
                tokens.push((Token::Op(BinOp::Div), i));
                i += 1;
            }
            '%' => {
                tokens.push((Token::Op(BinOp::Rem), i));
                i += 1;
            }
            '^' => {
                tokens.push((Token::Op(BinOp::Pow), i));
                i += 1;
            }
            '(' => {
                tokens.push((Token::LParen, i));
                i += 1;
            }
            ')' => {
                tokens.push((Token::RParen, i));
                i += 1;
            }
            ',' => {
                tokens.push((Token::Comma, i));
                i += 1;
            }
            _ => {
                return Err(ExprError::at(i, format!("无法识别的符号：{c}")));
            }
        }
    }
    if tokens.is_empty() {
        return Err(ExprError::at(0, "表达式为空"));
    }
    Ok(tokens)
}

/// Precedence climbing. Binary operator precedence rises with the value.
fn precedence(op: BinOp) -> u8 {
    match op {
        BinOp::Add | BinOp::Sub => 1,
        BinOp::Mul | BinOp::Div | BinOp::Rem => 2,
        BinOp::Pow => 3,
    }
}

struct Parser<'a> {
    tokens: &'a [Spanned],
    pos: usize,
}

impl Parser<'_> {
    fn peek(&self) -> Option<&Spanned> {
        self.tokens.get(self.pos)
    }

    fn expression(&mut self, min_precedence: u8) -> Result<Node, ExprError> {
        let mut lhs: Node = self.unary()?;
        // Left associative at every level, which is what makes `10 - 3 - 2` five rather than
        // nine. `^` is the exception and is handled in `unary`, which recurses for the
        // right-hand side.
        while let Some((Token::Op(op), _)) = self.peek() {
            if precedence(*op) < min_precedence {
                break;
            }
            let op = *op;
            self.pos += 1;
            // One more than this level's own precedence, so an equal-precedence operator
            // stops the loop and the remaining ones group to the right.
            let rhs = self.expression(precedence(op) + 1)?;
            lhs = Node::Binary {
                op,
                lhs: Box::new(lhs),
                rhs: Box::new(rhs),
            };
        }
        Ok(lhs)
    }

    fn unary(&mut self) -> Result<Node, ExprError> {
        match self.peek() {
            Some((Token::Op(BinOp::Sub), _)) => {
                self.pos += 1;
                Ok(Node::Neg(Box::new(self.unary()?)))
            }
            // A leading plus is a no-op rather than a negation, so `+3` is three.
            Some((Token::Op(BinOp::Add), _)) => {
                self.pos += 1;
                self.unary()
            }
            _ => self.power(),
        }
    }

    /// `atom`, optionally raised to a `unary` on the right.
    ///
    /// `^` sits here rather than in `expression` so it binds tighter than a leading minus
    /// (`-2^2` is `-(2^2)`, as it is written in a textbook) and so its right-hand side may
    /// itself be signed, which is what makes `2^-1` work.
    fn power(&mut self) -> Result<Node, ExprError> {
        let base = self.atom()?;
        if let Some((Token::Op(BinOp::Pow), _)) = self.peek() {
            self.pos += 1;
            let exponent = self.unary()?;
            return Ok(Node::Binary {
                op: BinOp::Pow,
                lhs: Box::new(base),
                rhs: Box::new(exponent),
            });
        }
        Ok(base)
    }

    fn atom(&mut self) -> Result<Node, ExprError> {
        let Some((token, offset)) = self.peek().cloned() else {
            return Err(ExprError::at(0, "表达式意外结束"));
        };
        self.pos += 1;
        match token {
            Token::Number(value) => Ok(Node::Number(value)),
            Token::LParen => {
                let inner = self.expression(0)?;
                match self.peek() {
                    Some((Token::RParen, _)) => {
                        self.pos += 1;
                        Ok(inner)
                    }
                    _ => Err(ExprError::at(offset, "括号没有闭合")),
                }
            }
            Token::Ident(name) => {
                if matches!(self.peek(), Some((Token::LParen, _))) {
                    self.pos += 1;
                    let expected = arity(&name)
                        .ok_or_else(|| ExprError::at(offset, format!("未知的函数：{name}")))?;
                    let mut args = Vec::with_capacity(expected);
                    loop {
                        args.push(self.expression(0)?);
                        match self.peek() {
                            Some((Token::Comma, _)) => self.pos += 1,
                            Some((Token::RParen, _)) => {
                                self.pos += 1;
                                break;
                            }
                            _ => return Err(ExprError::at(offset, "括号没有闭合")),
                        }
                    }
                    if args.len() != expected {
                        return Err(ExprError::at(
                            offset,
                            format!("{name} 需要 {expected} 个参数，但给了 {}", args.len()),
                        ));
                    }
                    // Interned rather than leaked: the table is fixed, so the name can be
                    // matched back to a `&'static str` and the expression stays `Clone`
                    // without a heap allocation per call.
                    let name: &'static str = KNOWN_FUNCTIONS
                        .iter()
                        .copied()
                        .find(|known| *known == name)
                        .ok_or_else(|| ExprError::at(offset, format!("未知的函数：{name}")))?;
                    Ok(Node::Call { name, args })
                } else {
                    match name.as_str() {
                        "t" => Ok(Node::Time),
                        "pi" => Ok(Node::Constant("pi")),
                        "e" => Ok(Node::Constant("e")),
                        "tau" => Ok(Node::Constant("tau")),
                        _ => Err(ExprError::at(offset, format!("未知的名称：{name}"))),
                    }
                }
            }
            Token::Op(op) => Err(ExprError::at(offset, format!("缺少左边的数：{op:?}"))),
            Token::Comma => Err(ExprError::at(offset, "多余的逗号")),
            Token::RParen => Err(ExprError::at(offset, "多余的右括号")),
        }
    }
}

/// Every function the parser will accept, so a name can be interned to a `&'static str`.
///
/// `arity` is the authority on which of these exist and how many arguments each takes; this
/// list exists only to give the names a lifetime longer than the call.
const KNOWN_FUNCTIONS: [&str; 27] = [
    "sin", "cos", "tan", "asin", "acos", "atan", "sinh", "cosh", "tanh", "exp", "ln", "log",
    "log10", "sqrt", "cbrt", "abs", "sign", "floor", "ceil", "round", "trunc", "sqr", "min", "max",
    "pow", "atan2", "hypot",
];

#[cfg(test)]
mod tests {
    use super::*;

    fn eval(text: &str, t: f64) -> f64 {
        Expr::parse(text)
            .unwrap_or_else(|err| panic!("{text:?} should parse: {err}"))
            .eval(t)
    }

    #[test]
    fn the_example_from_the_request_evaluates() {
        // 3 - sin(t - pi/2) is 3 + cos(t), since sin(t - pi/2) is -cos(t).
        let at_zero = eval("3 - sin(t - pi/2)", 0.0);
        assert!((at_zero - 4.0).abs() < 1e-12, "got {at_zero}");
        let at_quarter = eval("3 - sin(t - pi/2)", std::f64::consts::FRAC_PI_2);
        assert!((at_quarter - 3.0).abs() < 1e-12, "got {at_quarter}");
        let at_half = eval("3 - sin(t - pi/2)", std::f64::consts::PI);
        assert!((at_half - 2.0).abs() < 1e-12, "got {at_half}");
        // The parentheses have to change the meaning, or they are decoration.
        let t = 1.0;
        assert!((eval("(3 - sin(t)) / 2", t) - (3.0 - t.sin()) / 2.0).abs() < 1e-12);
        assert!((eval("3 - sin(t) / 2", t) - (3.0 - t.sin() / 2.0)).abs() < 1e-12);
    }

    #[test]
    fn arithmetic_binds_the_way_it_is_written() {
        // Left associative at the same precedence.
        assert!((eval("10 - 3 - 2", 0.0) - 5.0).abs() < 1e-12);
        assert!((eval("100 / 10 / 2", 0.0) - 5.0).abs() < 1e-12);
        // `^` binds tighter than a leading minus, and is right associative.
        assert!((eval("-2^2", 0.0) + 4.0).abs() < 1e-12);
        assert!((eval("2^3^2", 0.0) - 512.0).abs() < 1e-12);
        // A signed exponent, which is what makes `2^-1` work.
        assert!((eval("2^-1", 0.0) - 0.5).abs() < 1e-12);
        // Multiplication before subtraction.
        assert!((eval("2 + 3 * 4", 0.0) - 14.0).abs() < 1e-12);
    }

    #[test]
    fn the_time_variable_and_the_constants_are_available() {
        assert!((eval("t", 1.25) - 1.25).abs() < 1e-12);
        assert!((eval("pi", 0.0) - std::f64::consts::PI).abs() < 1e-12);
        assert!((eval("e", 0.0) - std::f64::consts::E).abs() < 1e-12);
        assert!((eval("tau", 0.0) - std::f64::consts::TAU).abs() < 1e-12);
        // A fifty hertz sine written the way a person would write it.
        let value = eval("5*sin(2*pi*50*t)", 0.005);
        assert!(
            (value - 5.0).abs() < 1e-9,
            "a quarter cycle in, got {value}"
        );
    }

    #[test]
    fn functions_take_the_number_of_arguments_they_are_given() {
        assert!((eval("max(3, 7)", 0.0) - 7.0).abs() < 1e-12);
        assert!((eval("min(3, 7)", 0.0) - 3.0).abs() < 1e-12);
        assert!((eval("pow(2, 10)", 0.0) - 1024.0).abs() < 1e-12);
        assert!((eval("abs(0 - 4)", 0.0) - 4.0).abs() < 1e-12);
        assert!((eval("sqr(5)", 0.0) - 25.0).abs() < 1e-12);
        assert!((eval("log(1000)", 0.0) - 3.0).abs() < 1e-12);
        assert!((eval("floor(2.7)", 0.0) - 2.0).abs() < 1e-12);
        assert_eq!(eval("sign(0)", 0.0), 0.0);
        assert_eq!(eval("sign(-0)", 0.0), 0.0);
        assert_eq!(eval("sign(-2)", 0.0), -1.0);
        assert_eq!(eval("sign(2)", 0.0), 1.0);
        // Case insensitive, because nobody remembers whether it is SIN or Sin.
        assert!((eval("SIN(0)", 0.0) - 0.0).abs() < 1e-12);
    }

    #[test]
    fn a_mistake_is_reported_where_it_was_written() {
        let err = Expr::parse("3 + sin(t").expect_err("unclosed paren");
        assert!(err.message.contains("括号"), "{err}");
        let err = Expr::parse("3 + ").expect_err("nothing after the plus");
        assert!(err.message.contains("结束"), "{err}");
        let err = Expr::parse("foo(2)").expect_err("no such function");
        assert!(err.message.contains("foo"), "{err}");
        let err = Expr::parse("nope").expect_err("no such name");
        assert!(err.message.contains("nope"), "{err}");
        let err = Expr::parse("sin(1, 2)").expect_err("sin takes one argument");
        assert!(err.message.contains("1"), "{err}");
        let err = Expr::parse("").expect_err("nothing at all");
        assert!(err.message.contains("空"), "{err}");
        let err = Expr::parse("1 @ 2").expect_err("not a symbol");
        assert!(err.message.contains('@'), "{err}");
        let err = Expr::parse("sin(t))").expect_err("a stray closing paren");
        assert!(err.message.contains("多余"), "{err}");
    }

    #[test]
    fn bad_arithmetic_reports_itself_rather_than_panicking() {
        // IEEE semantics: these come back as a non-finite value for the caller to notice.
        assert!(eval("1/0", 1.0).is_infinite());
        assert!(eval("ln(0 - 1)", 1.0).is_nan());
        assert!(eval("sqrt(0 - 4)", 1.0).is_nan());
        // And a good expression is unaffected.
        assert!((eval("2+2", 0.0) - 4.0).abs() < 1e-12);
    }

    #[test]
    fn nested_calls_and_deep_nesting_parse() {
        assert!((eval("sin(cos(sin(t)))", 0.0) - eval("sin(cos(0.0))", 0.0)).abs() < 1e-12);
        let deep = format!("{}+1", "sin(".repeat(24) + "0.0" + &")".repeat(24));
        Expr::parse(&deep).expect("deep nesting should be fine");
    }
}
