# Created by newuser for 5.8.1
# Created by newuser for 5.8

# ===== Zsh Essentials =====
autoload -U colors && colors
setopt prompt_subst
setopt auto_cd
setopt extended_glob
setopt correct
setopt autocd
setopt hist_ignore_dups
setopt share_history

# ===== Aliases =====
alias ll='ls -lah --color=auto'
alias la='ls -lA'
alias l='ls -CF'
alias gs='git status'
alias gp='git pull'
alias gd='git diff'
alias gco='git checkout'
alias ga='git add'
alias gc='git commit'
alias gb='git branch'
alias ..='cd ..'
alias ...='cd ../../'
alias ....='cd ../../../'
alias grep='grep --color=auto'
alias mv='mv -i'
alias cp='cp -i'
alias rm='rm -i'

# ===== Prompt =====
# Prefer Starship for a clean, professional prompt. Fallback to custom prompt if unavailable.
if command -v starship >/dev/null 2>&1; then
  eval "$(starship init zsh)"
else
  # Custom professional prompt with git support and execution time
  
  # Track command start time
  preexec() {
    cmd_start_time=$SECONDS
  }
  
  # Parse git branch with status
  parse_git_branch() {
    git branch 2>/dev/null | grep '*' | sed 's/* //'
  }
  
  # Get git status
  git_status() {
    local status=$(git status --porcelain 2>/dev/null)
    if [[ -n "$status" ]]; then
      echo "●"  # dirty
    else
      echo "✓"  # clean
    fi
  }
  
  # Calculate command execution time
  cmd_exec_time() {
    if [[ -n "$cmd_start_time" ]]; then
      local elapsed=$((SECONDS - cmd_start_time))
      if [[ $elapsed -gt 1 ]]; then
        if [[ $elapsed -lt 60 ]]; then
          echo " (${elapsed}s)"
        else
          echo " ($((elapsed / 60))m $((elapsed % 60))s)"
        fi
      fi
      unset cmd_start_time
    fi
  }
  
  # Git prompt info with icons
  git_prompt_info() {
    local branch=$(parse_git_branch)
    if [[ -n "$branch" ]]; then
      local status=$(git_status)
      echo " %F{blue}[${status} ${branch}]%f"
    fi
  }
  
  # Exit code indicator
  exit_code_indicator() {
    if [[ $? -eq 0 ]]; then
      echo "%F{green}✓%f"
    else
      echo "%F{red}✗%f"
    fi
  }
  
  # Main prompt
  PROMPT='
%F{cyan}┌─ %n%f %F{white}@%f %F{cyan}%m%f %F{white}|%f %F{yellow}%~%f$(git_prompt_info)
%F{cyan}└─%f $(exit_code_indicator) %F{white}❯%f '
  
  # Right side prompt with time and execution time
#   RPROMPT='%F{white}[%D{%H:%M:%S}]%f%(?)(%F{green}✓%f)($(cmd_exec_time)%F{red}✗%f)'
fi

# ===== Plugins =====
# Make sure these are installed: zsh-syntax-highlighting, zsh-autosuggestions
source /usr/share/zsh-autosuggestions/zsh-autosuggestions.zsh 2>/dev/null
source /usr/share/zsh-syntax-highlighting/zsh-syntax-highlighting.zsh 2>/dev/null

# ===== Completion =====
autoload -Uz compinit && compinit
zstyle ':completion:*' matcher-list 'm:{a-z}={A-Z}'
zstyle ':completion:*' list-colors "${(s.:.)LS_COLORS}"
zstyle ':completion:*' menu select=2

# ===== FZF Integration =====
if command -v fzf >/dev/null 2>&1; then
  eval "$(fzf --zsh)"
  
  # Ctrl+T for file search
  export FZF_CTRL_T_OPTS="--preview 'head -100 {}' --bind 'ctrl-/:toggle-preview'"
  
  # Ctrl+R for history with preview
  export FZF_CTRL_R_OPTS="--preview 'echo {}' --preview-window down:3:wrap"
  
  # Enhanced cd with FZF
  cd() {
    if [[ $# -eq 0 ]]; then
      builtin cd "$(find . -maxdepth 3 -type d -not -path '*/.*' | fzf --preview 'ls -lah {}')" || return
    else
      builtin cd "$@"
    fi
  }
fi

# ===== History =====
HISTFILE=~/.zsh_history
HISTSIZE=5000
SAVEHIST=5000

# ===== PATH tweaks =====
export PATH=$HOME/bin:/usr/local/bin:$PATH

# ===== Misc =====
export EDITOR=vim

# Safer file operations
setopt rm_star_wait
setopt no_clobber

# Better history behavior
setopt hist_find_no_dups
setopt hist_save_no_dups

# Case-insensitive path completion
zstyle ':completion:*' matcher-list 'm:{a-z}={A-Z}'

# Professional greeting
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Welcome back, $(whoami)! System ready at $(date '+%H:%M:%S')"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "already sourced: /opt/ros/humble/setup.zsh and ~/franka_ros2_ws/install/setup.zsh"
source /opt/ros/humble/setup.zsh
source ~/franka_ros2_ws/install/setup.zsh


source ~/workspaces/isaac_ros-dev/install/setup.zsh

# Isaac ROS environment
export ISAAC_ROS_WS=${HOME}/workspaces/isaac_ros-dev/
