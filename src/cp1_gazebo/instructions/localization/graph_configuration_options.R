options(warn=1)

args <- commandArgs(trailingOnly = TRUE)

cbind.fill <- function(...){
    nm <- list(...)
    nm <- lapply(nm, as.matrix)
    n <- max(sapply(nm, nrow))
    do.call(cbind, lapply(nm, function (x)
        rbind(x, matrix(, n-nrow(x), ncol(x)))))
}

option = args[1]
nfp = args[2]
default = args[3]
args = args[4:length(args)]
data_folder = "./data/"
plot_folder = "./plots/"
default_options_file = 'e26ab2de-93ae-11e6-bf5b-000c290c1bad_data.csv'
count = 0
reading_files = FALSE
servers = c()
options = c()
files = c()

i = 1
while (i < length(args))
{
    if (grepl(",", args[i]))
    {
        reading_files = TRUE
    }

    if (!reading_files){
        count = count + 1
    }

    if (reading_files)
    {
        options[i - count] <- as.numeric(sub(',', '', args[i]))
        files[i - count] <- sub(',', '', args[i + 1])
        i = i + 1
        count = count + 1
    }else
    {
       servers[i] <- args[i]
    }

    i = i + 1
}

server = 1
y_length = 6
while (server <= length(servers))
{
    quartz()
    x = c()
    y = c()
    e = c()
    server_name = servers[server]
    i = 1
    while (i <= length(files))
#    while (i <= y_length)
    {
        data = read.csv(paste(data_folder, files[i], sep=''))
        data = as.data.frame(data)
        data = data[[as.character(server_name)]]
        data = replace(data, data>=99, NA)
        x[i] <- options[i]
        y[i] <- mean(data, na.rm=TRUE)

        n <- sum(!is.na(data))

        if(n <= 1) {
            e[i]=0
        }
        else {
            sd <- sd(data, na.rm=TRUE)
            se = sd/sqrt(n)
            e[i] = qt(.995, df=n-1) * se
        }


        i = i + 1
    }

    y_total_max = c()
    y_total_min = c()
    i = 1
    while (i <= length(y))
    {
        y_total_max[i] <- y[i] + e[i]
        y_total_min[i] <- y[i] - e[i]
        i = i + 1
    }

#    default_data = read.csv(paste(data_folder, nfp, '_', default_options_file, sep=''))
#    default_data = as.data.frame(default_data)
#    default_data = default_data[[as.character(server_name)]]
#    default_data = replace(default_data, default_data>36, NA)

    max_y = max(y_total_max, na.rm=TRUE) + 0.025
    min_y = min(y_total_min, na.rm=TRUE) - 0.025

    png(file = paste(plot_folder, option, '_', nfp, '_', server_name, '.png', sep=''), width=1100,height=1100,res=150)
    par(mai=c(1.02,1.02,0.82,0.42))
    plot(x, y, pch=19, xlab="Option", ylab="Value", xlim=c(x[1], x[length(x)]), ylim=c(min_y, max_y))
#    axis(2, at=c(0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4))
#    plot(x, y, pch=19, xlab="option", ylab="value", xlim=c(x[1], x[y_length]))
    title(paste(option, '_', nfp, '_', server_name, sep=''))
    lines(x, y)
    arrows(x, y-e, x, y+e, length=0.05, angle=90, code=3)

    if(!default == 'NA')
    {
        abline(v=default, col='blue', lwd=3, lty=2)
    }

    dev.off()

    server = server + 1
}







