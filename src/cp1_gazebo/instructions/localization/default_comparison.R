# WARNING: This file might not work if the name of the documents do not match the ones we provide here

data_folder = "./data/"
plot_folder = "./plots/"
default_data_files = c('mean_amcl_cpu_utilization', 'mean_cpu_utilization',
                    'mean_localization_error', 'time')
default_options_file = 'e26ab2de-93ae-11e6-bf5b-000c290c1bad_data.csv'

for (i in 1:length(default_data_files)) {
    quartz()
    matrix = read.csv(paste(data_folder, default_data_files[i], '_', default_options_file, sep=''))

    invalid_data_indices = matrix > 36
    matrix[invalid_data_indices] = NA

    columns = colnames(matrix)

    png(file = paste(plot_folder, 'default_options_file', default_data_files[i], '.png', sep=''), width=1100,height=1100,res=150)
    boxplot(matrix, use.col=TRUE, las=2)
    title(default_data_files[i])
    means = c()
    for (j in 1:length(columns)) {
        server = as.character(columns[j])
        data = matrix[,server]
        means[j] <- mean(data, na.rm=TRUE)
    }

    points(means, col='red')
    abline(h=mean(means), col='red', lwd=3, lty=2)

    dev.off()
}